package dji.sampleV5.aircraft.rtsp

import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.util.Log
import java.nio.ByteBuffer
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Hardware H.264 encoder wrapper sized for low-latency live streaming.
 * Caller feeds NV12 frames at the configured resolution via [feedFrame];
 * the encoder emits SPS/PPS once via [NalListener.onConfig] and Annex B
 * NAL units thereafter via [NalListener.onNal].
 *
 * Runs synchronously on the caller's thread — designed to be driven from
 * the MSDK frame-listener thread without an intermediate queue. Frames are
 * dropped if no input buffer is available within 10 ms, which is better
 * than back-pressuring MSDK and stalling the camera pipeline.
 */
class H264Transcoder(
    private val width: Int,
    private val height: Int,
    private val bitrate: Int,
    private val frameRate: Int,
    private val iFrameIntervalSec: Int,
) {

    interface NalListener {
        fun onConfig(sps: ByteArray?, pps: ByteArray?)
        fun onNal(data: ByteArray, offset: Int, length: Int)
    }

    @Volatile var listener: NalListener? = null

    private var encoder: MediaCodec? = null
    private val running = AtomicBoolean(false)
    private val bufferInfo = MediaCodec.BufferInfo()
    @Volatile private var startNs: Long = 0L

    fun start(): Boolean {
        if (!running.compareAndSet(false, true)) return true
        return try {
            val format = MediaFormat.createVideoFormat(MIME, width, height).apply {
                setInteger(
                    MediaFormat.KEY_COLOR_FORMAT,
                    MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420SemiPlanar,
                )
                setInteger(MediaFormat.KEY_BIT_RATE, bitrate)
                setInteger(MediaFormat.KEY_FRAME_RATE, frameRate)
                setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, iFrameIntervalSec)
                setInteger(
                    MediaFormat.KEY_BITRATE_MODE,
                    MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR,
                )
            }
            val enc = MediaCodec.createEncoderByType(MIME)
            enc.configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
            enc.start()
            encoder = enc
            startNs = System.nanoTime()
            Log.i(TAG, "encoder started ${width}x${height} @ ${bitrate / 1000} kbps")
            true
        } catch (t: Throwable) {
            Log.e(TAG, "encoder start failed: ${t.message}", t)
            running.set(false)
            try { encoder?.release() } catch (_: Throwable) {}
            encoder = null
            false
        }
    }

    fun stop() {
        if (!running.compareAndSet(true, false)) return
        try { encoder?.stop() } catch (_: Throwable) {}
        try { encoder?.release() } catch (_: Throwable) {}
        encoder = null
    }

    fun isRunning(): Boolean = running.get()

    /**
     * Feed one NV12 frame at the configured resolution. The byte array is
     * copied into the encoder's input buffer before the call returns, so
     * the caller may reuse it immediately.
     */
    fun feedFrame(nv12: ByteArray) {
        if (!running.get()) return
        val codec = encoder ?: return
        try {
            // Drain output first to free up encoder pipeline slots.
            drainOutput(codec)
            val inIdx = codec.dequeueInputBuffer(10_000L)
            if (inIdx >= 0) {
                val buf = codec.getInputBuffer(inIdx)!!
                buf.clear()
                buf.put(nv12, 0, nv12.size)
                // Wall-clock pts (microseconds since encoder start) keeps us
                // honest if MSDK delivers frames at variable rate.
                val ptsUs = (System.nanoTime() - startNs) / 1_000L
                codec.queueInputBuffer(inIdx, 0, nv12.size, ptsUs, 0)
            }
            drainOutput(codec)
        } catch (t: Throwable) {
            Log.w(TAG, "feedFrame failed: ${t.message}")
        }
    }

    private fun drainOutput(codec: MediaCodec) {
        while (true) {
            val outIdx = codec.dequeueOutputBuffer(bufferInfo, 0)
            when {
                outIdx == MediaCodec.INFO_TRY_AGAIN_LATER -> return
                outIdx == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED -> {
                    handleFormatChange(codec.outputFormat)
                }
                outIdx >= 0 -> {
                    val outBuf = codec.getOutputBuffer(outIdx)
                    if (outBuf == null) {
                        codec.releaseOutputBuffer(outIdx, false)
                        continue
                    }
                    outBuf.position(bufferInfo.offset)
                    outBuf.limit(bufferInfo.offset + bufferInfo.size)
                    val bytes = ByteArray(bufferInfo.size)
                    outBuf.get(bytes)
                    handleOutput(bytes, bufferInfo.flags)
                    codec.releaseOutputBuffer(outIdx, false)
                }
            }
        }
    }

    private fun handleFormatChange(fmt: MediaFormat) {
        // Most encoders deliver SPS/PPS via csd-0/csd-1 when the output
        // format changes. Some also re-emit them inline with the
        // CODEC_CONFIG flag — handleOutput covers that path.
        val sps = fmt.getByteBuffer("csd-0")?.let { extractFirstNal(it) }
        val pps = fmt.getByteBuffer("csd-1")?.let { extractFirstNal(it) }
        if (sps != null || pps != null) {
            listener?.onConfig(sps, pps)
        }
    }

    private fun handleOutput(buf: ByteArray, flags: Int) {
        val lst = listener ?: return
        if ((flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG) != 0) {
            var sps: ByteArray? = null
            var pps: ByteArray? = null
            NalParser.parse(buf, 0, buf.size) { d, off, len ->
                if (len > 0) {
                    val type = NalParser.h264NalType(d[off])
                    val nal = d.copyOfRange(off, off + len)
                    when (type) {
                        NalParser.H264_NAL_SPS -> sps = nal
                        NalParser.H264_NAL_PPS -> pps = nal
                    }
                }
            }
            if (sps != null || pps != null) lst.onConfig(sps, pps)
            return
        }
        NalParser.parse(buf, 0, buf.size) { d, off, len ->
            lst.onNal(d, off, len)
        }
    }

    private fun extractFirstNal(buf: ByteBuffer): ByteArray? {
        buf.rewind()
        val bytes = ByteArray(buf.remaining())
        buf.get(bytes)
        var result: ByteArray? = null
        NalParser.parse(bytes, 0, bytes.size) { d, off, len ->
            if (result == null) result = d.copyOfRange(off, off + len)
        }
        return result
    }

    companion object {
        private const val TAG = "H264Transcoder"
        private const val MIME = MediaFormat.MIMETYPE_VIDEO_AVC
    }
}
