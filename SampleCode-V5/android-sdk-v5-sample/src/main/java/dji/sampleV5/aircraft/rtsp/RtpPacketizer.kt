package dji.sampleV5.aircraft.rtsp

/**
 * Packetises H.264 or H.265 NAL units into RTP packets and hands them to an
 * [RtpSender] (UDP datagram, or interleaved on the RTSP TCP socket). Per-client
 * state (sequence number, SSRC) lives here so each RTSP session gets its own
 * packetiser.
 *
 * H.264 follows RFC 6184: NALs under MTU go as a single NAL unit packet;
 * larger NALs use FU-A fragmentation.
 * H.265 follows RFC 7798: single NAL packets carry the 2-byte HEVC NAL header
 * verbatim; fragmentation uses FUs with a 3-byte header.
 *
 * Output is fire-and-forget — RTSP doesn't require RTCP feedback for a basic
 * CV/AI consumer to decode the stream.
 */
class RtpPacketizer(
    private val sender: RtpSender,
    private val codec: MainCameraSource.Codec,
    private val payloadType: Int = 96,
    private val ssrc: Int = (System.nanoTime() and 0xFFFFFFFF).toInt(),
) {

    @Volatile private var sequenceNumber: Int = (Math.random() * 65535).toInt() and 0xFFFF
    @Volatile var lastTimestamp: Long = 0L

    /**
     * Send the given NAL unit as one or more RTP packets, all sharing
     * [rtpTimestamp] (the 90 kHz video clock — see [timestamp90k]). Caller
     * should NOT include the Annex B start code in [nal].
     */
    fun sendNal(nal: ByteArray, offset: Int, length: Int, rtpTimestamp: Long, marker: Boolean) {
        if (length <= 0) return
        lastTimestamp = rtpTimestamp

        // -2 for the FU header in fragmented packets; pick the conservative
        // budget so the same MTU works for single-NAL and FU packets.
        val maxPayload = MAX_RTP_PAYLOAD - if (codec == MainCameraSource.Codec.H265) 3 else 2

        if (length <= MAX_RTP_PAYLOAD) {
            sendSingleNal(nal, offset, length, rtpTimestamp, marker)
        } else {
            if (codec == MainCameraSource.Codec.H265) {
                sendFuH265(nal, offset, length, rtpTimestamp, marker, maxPayload)
            } else {
                sendFuH264(nal, offset, length, rtpTimestamp, marker, maxPayload)
            }
        }
    }

    private fun sendSingleNal(nal: ByteArray, offset: Int, length: Int, ts: Long, marker: Boolean) {
        val packet = ByteArray(RTP_HEADER_SIZE + length)
        writeRtpHeader(packet, ts, marker)
        System.arraycopy(nal, offset, packet, RTP_HEADER_SIZE, length)
        send(packet)
    }

    private fun sendFuH264(nal: ByteArray, offset: Int, length: Int, ts: Long, marker: Boolean, maxPayload: Int) {
        // First byte of the NAL is the header — we extract its type bits and
        // reconstruct an FU indicator + FU header for each fragment.
        val nalHeader = nal[offset].toInt() and 0xFF
        val nri = nalHeader and 0x60        // bits 5..6 (NRI)
        val type = nalHeader and 0x1F       // bits 0..4
        val fuIndicator = (nri or 28).toByte()  // FU-A type = 28

        var remaining = length - 1
        var idx = offset + 1
        var first = true
        while (remaining > 0) {
            val chunk = minOf(maxPayload, remaining)
            val isLast = (chunk == remaining)
            val fuHeader = ((if (first) 0x80 else 0) or
                            (if (isLast) 0x40 else 0) or
                            type).toByte()

            val packet = ByteArray(RTP_HEADER_SIZE + 2 + chunk)
            writeRtpHeader(packet, ts, marker && isLast)
            packet[RTP_HEADER_SIZE]     = fuIndicator
            packet[RTP_HEADER_SIZE + 1] = fuHeader
            System.arraycopy(nal, idx, packet, RTP_HEADER_SIZE + 2, chunk)
            send(packet)

            idx += chunk
            remaining -= chunk
            first = false
        }
    }

    private fun sendFuH265(nal: ByteArray, offset: Int, length: Int, ts: Long, marker: Boolean, maxPayload: Int) {
        // HEVC has a 2-byte NAL header. Type lives in bits 1..6 of byte 0.
        val b0 = nal[offset].toInt() and 0xFF
        val b1 = nal[offset + 1].toInt() and 0xFF
        val type = (b0 and 0x7E) shr 1
        // PayloadHdr: replace type with 49 (FU), keep layer/tid bits.
        val payloadHdr0 = ((b0 and 0x81) or (49 shl 1)).toByte()
        val payloadHdr1 = b1.toByte()

        var remaining = length - 2
        var idx = offset + 2
        var first = true
        while (remaining > 0) {
            val chunk = minOf(maxPayload, remaining)
            val isLast = (chunk == remaining)
            val fuHeader = ((if (first) 0x80 else 0) or
                            (if (isLast) 0x40 else 0) or
                            type).toByte()

            val packet = ByteArray(RTP_HEADER_SIZE + 3 + chunk)
            writeRtpHeader(packet, ts, marker && isLast)
            packet[RTP_HEADER_SIZE]     = payloadHdr0
            packet[RTP_HEADER_SIZE + 1] = payloadHdr1
            packet[RTP_HEADER_SIZE + 2] = fuHeader
            System.arraycopy(nal, idx, packet, RTP_HEADER_SIZE + 3, chunk)
            send(packet)

            idx += chunk
            remaining -= chunk
            first = false
        }
    }

    private fun writeRtpHeader(packet: ByteArray, ts: Long, marker: Boolean) {
        packet[0] = 0x80.toByte()                              // V=2
        packet[1] = (payloadType or (if (marker) 0x80 else 0)).toByte()
        val seq = nextSeq()
        packet[2] = (seq ushr 8 and 0xFF).toByte()
        packet[3] = (seq and 0xFF).toByte()
        packet[4] = (ts ushr 24 and 0xFF).toByte()
        packet[5] = (ts ushr 16 and 0xFF).toByte()
        packet[6] = (ts ushr 8 and 0xFF).toByte()
        packet[7] = (ts and 0xFF).toByte()
        packet[8]  = (ssrc ushr 24 and 0xFF).toByte()
        packet[9]  = (ssrc ushr 16 and 0xFF).toByte()
        packet[10] = (ssrc ushr 8 and 0xFF).toByte()
        packet[11] = (ssrc and 0xFF).toByte()
    }

    private fun nextSeq(): Int {
        sequenceNumber = (sequenceNumber + 1) and 0xFFFF
        return sequenceNumber
    }

    private fun send(packet: ByteArray) {
        sender.send(packet)
    }

    companion object {
        private const val RTP_HEADER_SIZE = 12
        // Conservative MTU budget: 1500 (Ethernet) − 28 (IP+UDP) − 12 (RTP).
        // TCP-interleaved hits the same budget because the surrounding
        // 4-byte `$<ch><len>` framing doesn't change MTU math meaningfully.
        private const val MAX_RTP_PAYLOAD = 1400

        /** Convert a wall-clock ms duration since stream start to a 90 kHz RTP timestamp. */
        fun timestamp90k(elapsedMs: Long): Long = (elapsedMs * 90L) and 0xFFFFFFFFL
    }
}
