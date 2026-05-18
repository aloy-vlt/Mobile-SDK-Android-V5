package dji.sampleV5.aircraft.rtsp

/**
 * Scans a byte buffer for H.264/H.265 NAL units delimited by Annex B start
 * codes (`00 00 00 01` or `00 00 01`) and emits each NAL payload (without the
 * start code) to a callback.
 *
 * MSDK delivers encoded video as Annex B in chunks that may span NAL
 * boundaries; we keep no cross-call buffer here because each MSDK callback
 * typically contains one or more complete NALs starting with a start code.
 * If that assumption ever breaks we'd add a carry-over buffer.
 */
object NalParser {

    /**
     * Walk [data] from [offset] for [length] bytes, invoking [onNal] for each
     * NAL unit found (payload only, start code stripped).
     */
    inline fun parse(data: ByteArray, offset: Int, length: Int, onNal: (ByteArray, Int, Int) -> Unit) {
        val end = offset + length
        var i = offset
        var nalStart = -1
        while (i < end) {
            // Look for either 00 00 00 01 (4-byte) or 00 00 01 (3-byte).
            val startCodeLen = matchStartCode(data, i, end)
            if (startCodeLen > 0) {
                if (nalStart in 0 until i) {
                    onNal(data, nalStart, i - nalStart)
                }
                nalStart = i + startCodeLen
                i += startCodeLen
            } else {
                i++
            }
        }
        if (nalStart in 0 until end) {
            onNal(data, nalStart, end - nalStart)
        }
    }

    fun matchStartCode(data: ByteArray, i: Int, end: Int): Int {
        if (i + 3 < end &&
            data[i] == 0x00.toByte() && data[i + 1] == 0x00.toByte() &&
            data[i + 2] == 0x00.toByte() && data[i + 3] == 0x01.toByte()) return 4
        if (i + 2 < end &&
            data[i] == 0x00.toByte() && data[i + 1] == 0x00.toByte() &&
            data[i + 2] == 0x01.toByte()) return 3
        return 0
    }

    /** H.264 NAL unit type from the first byte of the NAL payload. */
    fun h264NalType(firstByte: Byte): Int = (firstByte.toInt() and 0x1F)

    /** H.265 NAL unit type from the first byte of the NAL payload (bits 1..6). */
    fun h265NalType(firstByte: Byte): Int = ((firstByte.toInt() and 0x7E) shr 1)

    // H.264 NAL types we care about.
    const val H264_NAL_NON_IDR = 1
    const val H264_NAL_IDR     = 5
    const val H264_NAL_SEI     = 6
    const val H264_NAL_SPS     = 7
    const val H264_NAL_PPS     = 8
    const val H264_NAL_AUD     = 9

    // H.265 NAL types we care about.
    const val H265_NAL_IDR_W_RADL = 19
    const val H265_NAL_IDR_N_LP   = 20
    const val H265_NAL_CRA        = 21
    const val H265_NAL_VPS        = 32
    const val H265_NAL_SPS        = 33
    const val H265_NAL_PPS        = 34
    const val H265_NAL_AUD        = 35
}
