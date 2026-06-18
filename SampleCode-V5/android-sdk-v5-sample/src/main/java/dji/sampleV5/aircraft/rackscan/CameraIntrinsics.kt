package dji.sampleV5.aircraft.rackscan

import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble

/**
 * DJI main-camera intrinsics from checkerboard calibration
 * (2026-06-18, tools/camera-calibration/calibrate.py — 9x6 inner corners,
 * 35 mm squares, mean reprojection error 0.39 px).
 *
 * Measured at 1920x1080. fx/fy/cx/cy are in pixels, so they scale linearly
 * with the frame resolution: if a live frame arrives at a different size than
 * the calibration size, [cameraMatrix] rescales them. Distortion coefficients
 * are normalised and resolution-independent.
 *
 * The measured fx implies a real HFOV of 2·atan(960/1387.95) ≈ 69°, NOT the
 * 82° the old apparent-size distance heuristic assumed — which is why that
 * heuristic was biased. ArUco pose via solvePnP uses this matrix directly and
 * is also tilt-robust, so it supersedes the heuristic.
 */
object CameraIntrinsics {
    const val CALIB_WIDTH = 1920
    const val CALIB_HEIGHT = 1080
    const val FX = 1387.9476
    const val FY = 1382.1718
    const val CX = 964.9026
    const val CY = 540.3764

    // k1, k2, p1, p2, k3
    private val DIST = doubleArrayOf(0.154879, -0.819674, 0.001386, 0.000910, 1.302979)

    /** Distortion coefficients as a 1x5 Mat (resolution-independent). */
    fun distCoeffs(): MatOfDouble = MatOfDouble(*DIST)

    /**
     * 3x3 camera matrix (CV_64F) for the given frame size, scaled from the
     * calibration resolution. Caller owns the returned Mat (release it).
     */
    fun cameraMatrix(frameWidth: Int, frameHeight: Int): Mat {
        val sx = frameWidth.toDouble() / CALIB_WIDTH
        val sy = frameHeight.toDouble() / CALIB_HEIGHT
        return Mat(3, 3, CvType.CV_64F).apply {
            put(0, 0,
                FX * sx, 0.0,     CX * sx,
                0.0,     FY * sy, CY * sy,
                0.0,     0.0,     1.0)
        }
    }
}
