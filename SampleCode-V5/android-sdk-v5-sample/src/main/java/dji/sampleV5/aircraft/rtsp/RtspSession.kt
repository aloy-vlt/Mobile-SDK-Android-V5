package dji.sampleV5.aircraft.rtsp

import android.util.Log
import java.util.concurrent.atomic.AtomicBoolean

/**
 * One per RTSP client. Created on SETUP, becomes an [MainCameraSource.NalConsumer]
 * on PLAY, unsubscribes on PAUSE / TEARDOWN / client disconnect.
 *
 * Each session owns its own [RtpPacketizer] (sequence numbers, SSRC) wired to
 * an [RtpSender] (UDP datagrams or TCP-interleaved on the RTSP control socket)
 * and its own stream-start time so multiple consumers can run concurrently
 * without colliding.
 */
class RtspSession(
    val id: String,
    private val source: MainCameraSource,
    private val sender: RtpSender,
    /** Free-form label for log lines — host:port for UDP, "tcp" for interleaved. */
    private val peerLabel: String,
) : MainCameraSource.NalConsumer {

    private val playing = AtomicBoolean(false)
    @Volatile private var startNs: Long = 0L
    private val packetizer = RtpPacketizer(
        sender = sender,
        codec = source.codec,
    )

    /** Begin streaming to this client. Idempotent. */
    fun start() {
        if (!playing.compareAndSet(false, true)) return
        startNs = System.nanoTime()
        // Front-load the parameter sets so the decoder can start on the first
        // slice instead of waiting for the next IDR.
        sendParameterSets()
        source.addNalConsumer(this)
        Log.i(TAG, "session $id PLAYING -> $peerLabel")
    }

    /** Stop streaming. RTSP control socket cleanup is the server's responsibility. */
    fun stop() {
        if (!playing.compareAndSet(true, false)) return
        source.removeNalConsumer(this)
        sender.close()
        Log.i(TAG, "session $id stopped")
    }

    fun isPlaying(): Boolean = playing.get()

    private fun nowTs90k(): Long {
        val elapsedMs = (System.nanoTime() - startNs) / 1_000_000L
        return RtpPacketizer.timestamp90k(elapsedMs)
    }

    private fun sendParameterSets() {
        val ts = nowTs90k()
        val codec = source.codec
        if (codec == MainCameraSource.Codec.H265) {
            source.vps?.let { packetizer.sendNal(it, 0, it.size, ts, false) }
            source.sps?.let { packetizer.sendNal(it, 0, it.size, ts, false) }
            source.pps?.let { packetizer.sendNal(it, 0, it.size, ts, false) }
        } else {
            source.sps?.let { packetizer.sendNal(it, 0, it.size, ts, false) }
            source.pps?.let { packetizer.sendNal(it, 0, it.size, ts, false) }
        }
    }

    override fun onNal(data: ByteArray, offset: Int, length: Int, nalType: Int) {
        if (!playing.get()) return
        val codec = source.codec

        // Parameter sets are forwarded out-of-band before every IDR; dropping
        // them here avoids double-transmission when the encoder also inlines
        // them right before the IDR.
        val isParameterSet = when (codec) {
            MainCameraSource.Codec.H264 ->
                nalType == NalParser.H264_NAL_SPS || nalType == NalParser.H264_NAL_PPS
            MainCameraSource.Codec.H265 ->
                nalType == NalParser.H265_NAL_VPS ||
                nalType == NalParser.H265_NAL_SPS ||
                nalType == NalParser.H265_NAL_PPS
            else -> false
        }
        if (isParameterSet) return

        // Most decoders don't need AUD and they bloat the RTP stream slightly.
        if (codec == MainCameraSource.Codec.H264 && nalType == NalParser.H264_NAL_AUD) return
        if (codec == MainCameraSource.Codec.H265 && nalType == NalParser.H265_NAL_AUD) return

        val isIdr = when (codec) {
            MainCameraSource.Codec.H264 -> nalType == NalParser.H264_NAL_IDR
            MainCameraSource.Codec.H265 -> nalType == NalParser.H265_NAL_IDR_W_RADL ||
                                           nalType == NalParser.H265_NAL_IDR_N_LP ||
                                           nalType == NalParser.H265_NAL_CRA
            else -> false
        }
        // VCL NAL types: H.264 1..5, H.265 0..31. Marker bit goes on the last
        // VCL NAL of an access unit; with MSDK's single-slice-per-frame output
        // every VCL NAL is also the last one of its AU.
        val isVcl = when (codec) {
            MainCameraSource.Codec.H264 -> nalType in 1..5
            MainCameraSource.Codec.H265 -> nalType in 0..31
            else -> false
        }

        val ts = nowTs90k()
        if (isIdr) sendParameterSets()
        try {
            packetizer.sendNal(data, offset, length, ts, marker = isVcl)
        } catch (t: Throwable) {
            Log.w(TAG, "session $id send failed: ${t.message}")
        }
    }

    companion object {
        private const val TAG = "RtspSession"
    }
}
