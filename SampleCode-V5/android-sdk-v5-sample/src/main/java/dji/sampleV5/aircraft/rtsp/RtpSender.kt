package dji.sampleV5.aircraft.rtsp

import android.util.Log
import java.io.OutputStream
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.Socket
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Transport-neutral target for RTP packets. Lets [RtpPacketizer] stay ignorant
 * of whether RTP is going out over UDP datagrams or interleaved on the RTSP
 * TCP control connection (RFC 2326 §10.12).
 *
 * Implementations MUST be fire-and-forget at the public API: dropping a packet
 * on overflow is preferable to blocking the encoder callback thread that drives
 * the whole consumer fan-out in [MainCameraSource].
 */
interface RtpSender {
    /** Best-effort send. Returns silently on failure / overflow. */
    fun send(packet: ByteArray)
    /** Release any owned resources (datagram socket, writer thread, etc.). */
    fun close()
}

/** RTP-over-UDP. Mirrors the prior [RtpPacketizer] inline UDP send path. */
class UdpRtpSender(
    private val socket: DatagramSocket,
    private val destAddr: InetAddress,
    private val destPort: Int,
) : RtpSender {

    override fun send(packet: ByteArray) {
        try {
            socket.send(DatagramPacket(packet, packet.size, destAddr, destPort))
        } catch (_: Throwable) { /* one dropped packet, continue */ }
    }

    override fun close() {
        try { socket.close() } catch (_: Throwable) {}
    }
}

/**
 * RTP-over-TCP interleaved (RFC 2326 §10.12). Each RTP packet is wrapped as
 * `$ <channel:u8> <length:u16-be> <rtp...>` on the same TCP socket carrying
 * RTSP signalling. Writes are serialized against text RTSP responses via
 * [writeMonitor] (typically the [Socket] instance itself).
 *
 * To avoid blocking the [MainCameraSource] encoder thread when the consumer
 * stalls, packets are queued onto a bounded ring and drained by a dedicated
 * writer thread. Queue overflow drops oldest first — matching UDP's lossy
 * semantics so the source thread never stalls on a slow client.
 */
class TcpInterleavedRtpSender(
    private val out: OutputStream,
    private val writeMonitor: Any,
    private val rtpChannel: Int,
    private val sessionId: String,
    queueCapacity: Int = 512,
) : RtpSender {

    private val queue = ArrayBlockingQueue<ByteArray>(queueCapacity)
    private val running = AtomicBoolean(true)
    private val writer = Thread({ writerLoop() }, "rtsp-tcp-rtp-$sessionId").apply {
        isDaemon = true
        start()
    }

    override fun send(packet: ByteArray) {
        if (!running.get()) return
        // Non-blocking: if the consumer can't keep up, evict the oldest queued
        // packet so the freshest data still gets a chance to flow. This favors
        // current frames over historical ones.
        if (!queue.offer(packet)) {
            queue.poll()
            queue.offer(packet)
        }
    }

    override fun close() {
        if (!running.compareAndSet(true, false)) return
        // Wake the writer if it's blocked on take().
        writer.interrupt()
    }

    private fun writerLoop() {
        val header = ByteArray(4)
        header[0] = 0x24                                  // '$'
        header[1] = (rtpChannel and 0xFF).toByte()
        try {
            while (running.get()) {
                val packet = try {
                    queue.poll(1, TimeUnit.SECONDS) ?: continue
                } catch (_: InterruptedException) {
                    break
                }
                val len = packet.size
                header[2] = ((len ushr 8) and 0xFF).toByte()
                header[3] = (len and 0xFF).toByte()
                try {
                    synchronized(writeMonitor) {
                        out.write(header)
                        out.write(packet)
                        out.flush()
                    }
                } catch (t: Throwable) {
                    Log.w(TAG, "session $sessionId TCP RTP write failed: ${t.message}")
                    // Socket is gone — drop the running flag and exit. The
                    // RtspServer client loop will clean up the session.
                    running.set(false)
                    break
                }
            }
        } finally {
            queue.clear()
        }
    }

    companion object {
        private const val TAG = "TcpRtpSender"
    }
}
