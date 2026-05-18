package dji.sampleV5.aircraft.rtsp

import android.util.Base64
import android.util.Log
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import java.util.concurrent.CopyOnWriteArraySet
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Re-encodes MSDK's main-camera NV21 frames (`LEFT_OR_MAIN`) into H.264 at a
 * reduced resolution / bitrate, then fans out the resulting NAL units to
 * registered consumers (one per RTSP client).
 *
 * Why not passthrough? MSDK delivers the camera's native encoded stream
 * (typically 1080p at 8-15 Mbps), which is too fat for a Wi-Fi LAN hop to
 * a laptop and shows up as jitter / I-frame concealment errors in players.
 * Transcoding to 720p at a few Mbps costs ~30-50 ms of encode latency but
 * is a far smaller cost than the alternative network lag.
 *
 * The dashboard's own NV21 listener still works alongside this one — MSDK
 * fans out frame listeners independently per call site.
 */
class MainCameraSource(
    val targetWidth: Int = DEFAULT_WIDTH,
    val targetHeight: Int = DEFAULT_HEIGHT,
    val targetBitrate: Int = DEFAULT_BITRATE,
    val targetFps: Int = DEFAULT_FPS,
    val gopSec: Int = DEFAULT_GOP_SEC,
) {

    interface NalConsumer {
        /** [data] is reused by the encoder pipeline — copy before queueing. */
        fun onNal(data: ByteArray, offset: Int, length: Int, nalType: Int)
    }

    enum class Codec { H264, H265, UNKNOWN }

    /** Always H.264 since the transcoder targets AVC. Kept for SDP path symmetry. */
    val codec: Codec = Codec.H264

    @Volatile var sps: ByteArray? = null
        private set
    @Volatile var pps: ByteArray? = null
        private set
    /** H.265-only field. Always null on this source — retained so the SDP builder compiles. */
    val vps: ByteArray? = null

    val width: Int get() = targetWidth
    val height: Int get() = targetHeight

    private val consumers = CopyOnWriteArraySet<NalConsumer>()
    private val started = AtomicBoolean(false)
    private val cameraIndex = ComponentIndexType.LEFT_OR_MAIN

    private var transcoder: H264Transcoder? = null
    @Volatile private var scratch: ByteArray? = null

    private val transcoderListener = object : H264Transcoder.NalListener {
        override fun onConfig(sps: ByteArray?, pps: ByteArray?) {
            sps?.let { this@MainCameraSource.sps = it }
            pps?.let { this@MainCameraSource.pps = it }
            Log.i(TAG, "encoder config: sps=${sps?.size ?: 0}B pps=${pps?.size ?: 0}B")
        }
        override fun onNal(data: ByteArray, offset: Int, length: Int) {
            if (length <= 0) return
            val nalType = NalParser.h264NalType(data[offset])
            // Mirror SPS/PPS that arrive inline (some encoders re-emit them
            // before every IDR even after csd-0 / csd-1) so DESCRIBE always
            // sees the freshest set.
            when (nalType) {
                NalParser.H264_NAL_SPS -> this@MainCameraSource.sps = data.copyOfRange(offset, offset + length)
                NalParser.H264_NAL_PPS -> this@MainCameraSource.pps = data.copyOfRange(offset, offset + length)
            }
            for (c in consumers) {
                try {
                    c.onNal(data, offset, length, nalType)
                } catch (t: Throwable) {
                    Log.w(TAG, "consumer threw on NAL: ${t.message}")
                }
            }
        }
    }

    private val frameListener = ICameraStreamManager.CameraFrameListener {
            frameData, offset, length, width, height, _ ->
        val tc = transcoder ?: return@CameraFrameListener
        val expected = targetWidth * targetHeight * 3 / 2
        var buf = scratch
        if (buf == null || buf.size < expected) {
            buf = ByteArray(expected)
            scratch = buf
        }
        try {
            // Down-scale + NV21->NV12 in one pass into the scratch buffer.
            // Bypasses a copy round-trip through the encoder when input
            // resolution already matches the target.
            if (width == targetWidth && height == targetHeight) {
                // Still need NV21 -> NV12 byte swap.
                System.arraycopy(frameData, offset, buf, 0, targetWidth * targetHeight)
                val uvBase = targetWidth * targetHeight
                val uvBytes = length - uvBase
                var i = 0
                while (i + 1 < uvBytes) {
                    buf[uvBase + i]     = frameData[offset + uvBase + i + 1]
                    buf[uvBase + i + 1] = frameData[offset + uvBase + i]
                    i += 2
                }
            } else {
                Nv21Scaler.scaleAndConvertToNv12(
                    frameData, offset, width, height,
                    buf, targetWidth, targetHeight,
                )
            }
            tc.feedFrame(buf)
        } catch (t: Throwable) {
            Log.w(TAG, "transcode failed: ${t.message}")
        }
    }

    fun start(): Boolean {
        if (!started.compareAndSet(false, true)) return true
        return try {
            val tc = H264Transcoder(
                width = targetWidth,
                height = targetHeight,
                bitrate = targetBitrate,
                frameRate = targetFps,
                iFrameIntervalSec = gopSec,
            ).apply { listener = transcoderListener }
            if (!tc.start()) {
                started.set(false)
                return false
            }
            transcoder = tc

            MediaDataCenter.getInstance().cameraStreamManager.addFrameListener(
                cameraIndex,
                ICameraStreamManager.FrameFormat.NV21,
                frameListener,
            )
            Log.i(TAG, "MainCameraSource started: transcoding to ${targetWidth}x${targetHeight} @ ${targetBitrate / 1000} kbps")
            true
        } catch (t: Throwable) {
            started.set(false)
            try { transcoder?.stop() } catch (_: Throwable) {}
            transcoder = null
            Log.e(TAG, "failed to start camera source: ${t.message}", t)
            false
        }
    }

    fun stop() {
        if (!started.compareAndSet(true, false)) return
        try {
            MediaDataCenter.getInstance().cameraStreamManager.removeFrameListener(frameListener)
        } catch (_: Throwable) { /* fine */ }
        try { transcoder?.stop() } catch (_: Throwable) {}
        transcoder = null
        consumers.clear()
        sps = null; pps = null
        scratch = null
    }

    fun addNalConsumer(c: NalConsumer) { consumers.add(c) }
    fun removeNalConsumer(c: NalConsumer) { consumers.remove(c) }

    fun spsBase64(): String? = sps?.let { Base64.encodeToString(it, Base64.NO_WRAP) }
    fun ppsBase64(): String? = pps?.let { Base64.encodeToString(it, Base64.NO_WRAP) }
    fun vpsBase64(): String? = null

    companion object {
        private const val TAG = "MainCameraSource"
        const val DEFAULT_WIDTH = 1280
        const val DEFAULT_HEIGHT = 720
        const val DEFAULT_BITRATE = 3_000_000   // 3 Mbps — ~5x reduction vs native 1080p
        const val DEFAULT_FPS = 30
        const val DEFAULT_GOP_SEC = 2
    }
}
