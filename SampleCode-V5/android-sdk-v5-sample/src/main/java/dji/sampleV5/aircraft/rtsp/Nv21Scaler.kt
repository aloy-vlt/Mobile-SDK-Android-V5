package dji.sampleV5.aircraft.rtsp

/**
 * Nearest-neighbour NV21 → NV12 scaler. Single pass over source pixels;
 * caller-supplied destination buffer to avoid per-frame allocation.
 *
 * NV21 layout: Y plane (w*h bytes) followed by interleaved VU pairs (V first).
 * NV12 layout: Y plane (w*h bytes) followed by interleaved UV pairs (U first).
 *
 * The byte swap matters: most Android H.264 encoders accept
 * COLOR_FormatYUV420SemiPlanar but disagree on chroma order. NV12 (U,V) is
 * the spec-compliant interpretation and is accepted by every major SoC.
 */
object Nv21Scaler {

    /**
     * Scale [src] (NV21 at [srcW]x[srcH], starting at [srcOffset]) into
     * [dst] (NV12 at [dstW]x[dstH], starting at byte 0). [dst] must be at
     * least `dstW * dstH * 3 / 2` bytes long.
     */
    fun scaleAndConvertToNv12(
        src: ByteArray, srcOffset: Int, srcW: Int, srcH: Int,
        dst: ByteArray, dstW: Int, dstH: Int,
    ) {
        // 16.16 fixed-point ratios let us avoid floating-point math in the
        // hot loop and still keep sub-pixel sampling accurate enough for
        // nearest-neighbour video.
        val xRatio = (srcW shl 16) / dstW
        val yRatio = (srcH shl 16) / dstH

        // Y plane.
        for (y in 0 until dstH) {
            val srcY = (y * yRatio) shr 16
            val srcRowOff = srcOffset + srcY * srcW
            val dstRowOff = y * dstW
            for (x in 0 until dstW) {
                val srcX = (x * xRatio) shr 16
                dst[dstRowOff + x] = src[srcRowOff + srcX]
            }
        }

        // VU/UV plane (chroma is 2:1 subsampled in both axes for 4:2:0).
        val srcChromaW = srcW / 2
        val srcChromaH = srcH / 2
        val dstChromaW = dstW / 2
        val dstChromaH = dstH / 2
        val xRatioC = (srcChromaW shl 16) / dstChromaW
        val yRatioC = (srcChromaH shl 16) / dstChromaH
        val srcUvBase = srcOffset + srcW * srcH
        val dstUvBase = dstW * dstH
        for (y in 0 until dstChromaH) {
            val srcY = (y * yRatioC) shr 16
            val srcRowOff = srcUvBase + srcY * srcChromaW * 2
            val dstRowOff = dstUvBase + y * dstChromaW * 2
            for (x in 0 until dstChromaW) {
                val srcX = ((x * xRatioC) shr 16) * 2
                // NV21 stores (V, U); NV12 wants (U, V) — swap on copy.
                dst[dstRowOff + x * 2]     = src[srcRowOff + srcX + 1]
                dst[dstRowOff + x * 2 + 1] = src[srcRowOff + srcX]
            }
        }
    }
}
