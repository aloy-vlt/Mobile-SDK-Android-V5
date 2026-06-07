package dji.sampleV5.aircraft.rackmission

import android.util.Log
import dji.sampleV5.aircraft.rackscan.MissionExecutor
import dji.sampleV5.aircraft.rackscan.RackScanLogBuffer
import dji.sampleV5.aircraft.rackscan.RackScanTelemetry
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.et.action
import dji.v5.et.create
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager
import org.opencv.android.OpenCVLoader
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.objdetect.ArucoDetector
import org.opencv.objdetect.DetectorParameters
import org.opencv.objdetect.Objdetect
import java.util.Timer
import java.util.TimerTask
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.abs

/**
 * Real flight control for the Rack Scan Missioning module — the [MissionExecutor.Host]
 * implementation. Same technique as [dji.sampleV5.aircraft.pages.ArucoFollowFragment]:
 * a 20 Hz `sendVirtualStickAdvancedParam` pump that the mission state drives.
 *
 *   - LEFT / RIGHT sweeps → VELOCITY mode (BODY frame), pitch axis = ±sweep speed.
 *   - UP / DOWN climbs → POSITION mode, verticalThrottle = absolute target altitude.
 *   - ALIGN (ArUco) → ANGLE mode, roll = PD output that centers the marker. This is
 *     the GPS-denied indoor positioning step: it re-centers on a marker so the lateral
 *     drift accumulated by open-loop time-based sweeps is corrected before the next step.
 *   - HOVER / between steps → ANGLE roll/pitch = 0, vertical VELOCITY 0.
 *
 * ArUco centering mirrors [dji.sampleV5.aircraft.models.DashboardServerVM]: detection
 * runs on the camera's NV21 Y-plane (fed in by the ViewModel via [processAlignFrame]),
 * a PD controller turns the marker's horizontal offset into a roll-angle command, and
 * the pump flushes it at 20 Hz. Only horizontal centering is done (the axis that drift
 * affects during lateral sweeps); altitude is owned by the climb steps.
 */
class RackFlightController(
    private val telemetry: RackScanTelemetry,
    private val logs: RackScanLogBuffer,
) : MissionExecutor.Host {

    enum class FlightState { IDLE, HOVER, SWEEP_LEFT, SWEEP_RIGHT, CLIMB_UP, CLIMB_DOWN, ALIGN }

    @Volatile var sweepSpeedMps: Float = DEFAULT_SWEEP_SPEED_MPS
    @Volatile private var state: FlightState = FlightState.IDLE
    @Volatile private var vsEnabled = false
    @Volatile private var climbStartAltitudeM = 0.0
    @Volatile private var climbDistanceM = 0f

    // ── ArUco align state ──
    @Volatile private var arucoDetector: ArucoDetector? = null
    @Volatile private var arucoReady = false
    @Volatile private var targetMarkerId = 0
    @Volatile private var targetRollDeg = 0f      // lateral centering (roll, degrees)
    @Volatile private var targetVz = 0f           // elevation centering (vertical velocity, m/s)
    @Volatile private var prevOffsetX = 0f
    @Volatile private var prevOffsetY = 0f
    @Volatile private var inHoverLock = true
    @Volatile private var kickEndsAtMs = 0L
    @Volatile private var centered = false        // true only when centered on BOTH axes
    @Volatile private var wasDetected = false     // edge-detect so we log acquisition once, not per frame

    private var pumpTimer: Timer? = null
    private val pumpTicks = AtomicLong(0)

    fun start() {
        if (pumpTimer != null) return
        telemetry.sweepSpeedMps = sweepSpeedMps
        pumpTimer = Timer("RackMissionPump", true).also {
            it.scheduleAtFixedRate(object : TimerTask() {
                override fun run() = pumpTick()
            }, 0L, COMMAND_INTERVAL_MS)
        }
    }

    fun shutdown() {
        pumpTimer?.cancel(); pumpTimer = null
        if (vsEnabled) disableVS()
    }

    fun setSweepSpeed(mps: Float) {
        sweepSpeedMps = mps.coerceIn(MIN_SWEEP_MPS, MAX_SWEEP_MPS)
        telemetry.sweepSpeedMps = sweepSpeedMps
        logs.i(TAG, "Sweep speed set to ${"%.2f".format(sweepSpeedMps)} m/s")
    }

    fun isVSEnabled(): Boolean = vsEnabled
    fun isAligning(): Boolean = state == FlightState.ALIGN

    // ── Operator actions (called off-main from the command thread) ──────

    fun takeoff() {
        FlightControllerKey.KeyStartTakeoff.create().action({
            logs.i(TAG, "Takeoff command accepted")
        }, { e: IDJIError -> logs.e(TAG, "Takeoff failed: $e") })
    }

    fun enableVS() {
        if (vsEnabled) return
        VirtualStickManager.getInstance().enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                try { VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(true) }
                catch (t: Throwable) { Log.w(TAG, "adv mode: ${t.message}") }
                vsEnabled = true
                telemetry.isVSEnabled = true
                state = FlightState.HOVER
                logs.i(TAG, "Virtual Stick enabled (advanced mode)")
            }
            override fun onFailure(error: IDJIError) { logs.e(TAG, "VS enable failed: $error") }
        })
    }

    fun disableVS() {
        state = FlightState.IDLE
        try { VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(false) } catch (_: Throwable) {}
        VirtualStickManager.getInstance().disableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { vsEnabled = false; telemetry.isVSEnabled = false; logs.i(TAG, "Virtual Stick disabled") }
            override fun onFailure(error: IDJIError) { logs.w(TAG, "VS disable failed: $error") }
        })
    }

    fun land() {
        state = FlightState.IDLE
        try { VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(false) } catch (_: Throwable) {}
        VirtualStickManager.getInstance().disableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { vsEnabled = false; telemetry.isVSEnabled = false; startLanding() }
            override fun onFailure(error: IDJIError) { startLanding() }
        })
    }

    private fun startLanding() {
        FlightControllerKey.KeyStartAutoLanding.create().action({
            logs.i(TAG, "Landing started")
        }, { e: IDJIError -> logs.e(TAG, "Landing failed: $e") })
    }

    // ── MissionExecutor.Host ────────────────────────────────────────────

    override fun currentAltitudeM(): Double = telemetry.altitudeM
    override fun motorsOn(): Boolean = telemetry.motorsOn

    override fun missionStartLeft()  { state = FlightState.SWEEP_LEFT }
    override fun missionStartRight() { state = FlightState.SWEEP_RIGHT }

    override fun missionStartUp(distanceM: Float) {
        climbStartAltitudeM = telemetry.altitudeM; climbDistanceM = distanceM; state = FlightState.CLIMB_UP
    }
    override fun missionStartDown(distanceM: Float) {
        climbStartAltitudeM = telemetry.altitudeM; climbDistanceM = distanceM; state = FlightState.CLIMB_DOWN
    }

    override fun missionStartAlign(markerId: Int) {
        targetMarkerId = markerId
        if (!arucoReady) {
            if (!OpenCVLoader.initLocal()) { logs.e(TAG, "OpenCV init failed — cannot align"); return }
            arucoDetector = ArucoDetector(Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50), DetectorParameters())
            arucoReady = true
            logs.i(TAG, "ArUco detector ready (DICT_4X4_50)")
        }
        prevOffsetX = 0f; prevOffsetY = 0f; inHoverLock = true; kickEndsAtMs = 0L
        centered = false; targetRollDeg = 0f; targetVz = 0f; wasDetected = false
        telemetry.currentTargetId = markerId; telemetry.isTracking = true
        state = FlightState.ALIGN
        logs.i(TAG, "Aligning to ArUco marker $markerId")
    }

    override fun isCentered(): Boolean = state == FlightState.ALIGN && centered

    override fun missionStopAndHover() {
        centered = false; targetRollDeg = 0f; targetVz = 0f; wasDetected = false
        telemetry.isTracking = false; telemetry.detected = false; telemetry.alignCentered = false
        state = if (vsEnabled) FlightState.HOVER else FlightState.IDLE
    }

    override fun missionLand() { land() }

    // ── ArUco frame processing (called from the ViewModel's encoder thread) ──

    /** Detect the target marker in [nv21] and update the centering command +
     *  [centered] flag. No-op unless an ALIGN step is active. */
    fun processAlignFrame(nv21: ByteArray, width: Int, height: Int) {
        if (state != FlightState.ALIGN) return
        val det = arucoDetector ?: return
        // ArUco needs only the grayscale Y plane; NV21's first w*h bytes are exactly that.
        val yuvMat = Mat(height + height / 2, width, CvType.CV_8UC1)
        yuvMat.put(0, 0, nv21)
        val gray = yuvMat.submat(0, height, 0, width)
        val corners = mutableListOf<Mat>()
        val ids = Mat()
        try {
            det.detectMarkers(gray, corners, ids)
            var detected = false
            var offsetX = 0f
            var offsetY = 0f
            if (ids.rows() > 0) {
                for (i in 0 until ids.rows()) {
                    if (ids[i, 0][0].toInt() == targetMarkerId) {
                        detected = true
                        val mc = corners[i]
                        var sx = 0.0; var sy = 0.0
                        for (j in 0 until 4) { sx += mc[0, j][0]; sy += mc[0, j][1] }
                        val cx = (sx / 4.0).toFloat(); val cy = (sy / 4.0).toFloat()
                        offsetX = (cx - width / 2f) / (width / 2f)
                        offsetY = (cy - height / 2f) / (height / 2f)   // >0 = marker below frame centre
                        break
                    }
                }
            }
            if (detected) {
                targetRollDeg = computeRoll(offsetX)        // lateral
                targetVz      = computeVertical(offsetY)    // elevation
                centered = abs(offsetX) < DEAD_ZONE && abs(offsetY) < DEAD_ZONE_Y
            } else {
                targetRollDeg = 0f; targetVz = 0f; centered = false
                prevOffsetX = 0f; prevOffsetY = 0f; inHoverLock = true
            }
            // Log acquisition/loss on the edge only (this runs every frame).
            if (detected != wasDetected) {
                if (detected) logs.i(TAG, "[DETECTED]: Aruco ID: $targetMarkerId detected.")
                else          logs.w(TAG, "[LOST]: Aruco ID: $targetMarkerId out of view.")
                wasDetected = detected
            }
            telemetry.detected = detected
            telemetry.offsetX = offsetX
            telemetry.offsetY = offsetY
            telemetry.alignCentered = centered
        } catch (t: Throwable) {
            Log.w(TAG, "align detect: ${t.message}")
        } finally {
            gray.release(); ids.release(); corners.forEach { it.release() }; yuvMat.release()
        }
    }

    /** PD → roll angle (degrees), with a kickstart out of hover-lock so the FC's
     *  noise filter doesn't swallow the first small correction. Mirrors
     *  DashboardServerVM.computeRollAngle. */
    private fun computeRoll(offsetX: Float): Float {
        if (abs(offsetX) < DEAD_ZONE) { prevOffsetX = offsetX; inHoverLock = true; return 0f }
        val pTerm = P_GAIN * offsetX
        val dTerm = D_GAIN * (offsetX - prevOffsetX)
        prevOffsetX = offsetX
        var ratio = pTerm + dTerm
        if (inHoverLock) { inHoverLock = false; kickEndsAtMs = System.currentTimeMillis() + KICK_DURATION_MS }
        if (System.currentTimeMillis() < kickEndsAtMs && abs(ratio) > 0f && abs(ratio) < KICK_RATIO)
            ratio = if (ratio > 0) KICK_RATIO else -KICK_RATIO
        ratio = ratio.coerceIn(-MAX_ROLL_RATIO, MAX_ROLL_RATIO)
        return ratio * MAX_ROLL_ANGLE_DEG
    }

    /** PD → vertical velocity (m/s) to center the marker's elevation. Marker
     *  below frame centre (offsetY > 0) → descend (negative). A small floor
     *  beats the FC velocity filter on the last bit of centering, mirroring the
     *  roll kickstart. */
    private fun computeVertical(offsetY: Float): Float {
        if (abs(offsetY) < DEAD_ZONE_Y) { prevOffsetY = offsetY; return 0f }
        val p = V_P_GAIN * offsetY
        val d = V_D_GAIN * (offsetY - prevOffsetY)
        prevOffsetY = offsetY
        var v = -(p + d)
        if (v != 0f && abs(v) < MIN_V_MPS) v = if (v > 0) MIN_V_MPS else -MIN_V_MPS
        return v.coerceIn(-MAX_V_MPS, MAX_V_MPS)
    }

    // ── 20 Hz pump ──────────────────────────────────────────────────────

    private fun pumpTick() {
        val s = state
        if (!vsEnabled || s == FlightState.IDLE) return
        val sweeping = s == FlightState.SWEEP_LEFT || s == FlightState.SWEEP_RIGHT
        val climbing = s == FlightState.CLIMB_UP || s == FlightState.CLIMB_DOWN
        val verticalMode = if (climbing) VerticalControlMode.POSITION else VerticalControlMode.VELOCITY
        val verticalCommand: Double = when (s) {
            FlightState.CLIMB_UP   -> climbStartAltitudeM + climbDistanceM
            FlightState.CLIMB_DOWN -> (climbStartAltitudeM - climbDistanceM).coerceAtLeast(0.30)
            FlightState.ALIGN      -> targetVz.toDouble()   // VELOCITY-mode elevation centering
            else                   -> 0.0
        }
        try {
            val param = VirtualStickFlightControlParam().apply {
                rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                yawControlMode            = YawControlMode.ANGULAR_VELOCITY
                verticalControlMode       = verticalMode
                yaw                       = 0.0
                verticalThrottle          = verticalCommand
                if (sweeping) {
                    rollPitchControlMode = RollPitchControlMode.VELOCITY
                    val dir = if (s == FlightState.SWEEP_RIGHT) 1.0 else -1.0
                    pitch = sweepSpeedMps.toDouble() * dir
                    roll  = 0.0
                } else {
                    // HOVER and ALIGN both use ANGLE; ALIGN drives roll to center the marker.
                    rollPitchControlMode = RollPitchControlMode.ANGLE
                    pitch = 0.0
                    roll  = if (s == FlightState.ALIGN) targetRollDeg.toDouble() else 0.0
                }
            }
            VirtualStickManager.getInstance().sendVirtualStickAdvancedParam(param)
            pumpTicks.incrementAndGet()

            telemetry.stateName            = s.name
            telemetry.rollPitchControlMode = if (sweeping) "VELOCITY" else "ANGLE"
            telemetry.verticalControlMode  = verticalMode.name
            telemetry.rollDeg              = param.roll
            telemetry.pitchMps             = param.pitch
            telemetry.verticalThrottleMps  = param.verticalThrottle
            telemetry.pumpTicks            = pumpTicks.get()
            telemetry.sweepSpeedMps        = sweepSpeedMps
            telemetry.inHoverLock          = inHoverLock
            telemetry.inKick               = System.currentTimeMillis() < kickEndsAtMs
        } catch (t: Throwable) {
            Log.w(TAG, "pump: ${t.message}")
        }
    }

    companion object {
        private const val TAG = "RackFlight"
        private const val COMMAND_INTERVAL_MS = 50L         // 20 Hz
        private const val DEFAULT_SWEEP_SPEED_MPS = 0.30f   // gentle indoor sweep
        private const val MIN_SWEEP_MPS = 0.05f
        private const val MAX_SWEEP_MPS = 1.0f
        // ArUco centering (mirrors DashboardServerVM constants)
        // DEAD_ZONE = "close enough" tolerance: bigger = stops correcting sooner
        // (less fidgeting), smaller = tries harder to perfectly centre.
        private const val DEAD_ZONE = 0.10f
        private const val MAX_ROLL_RATIO = 0.6f
        private const val MAX_ROLL_ANGLE_DEG = 15.0f
        private const val KICK_RATIO = 0.25f
        private const val KICK_DURATION_MS = 250L
        private const val P_GAIN = 0.15f
        private const val D_GAIN = 0.10f
        // Vertical (elevation) centering — VELOCITY-mode m/s, gentle for indoor.
        private const val DEAD_ZONE_Y = 0.12f
        private const val V_P_GAIN = 0.6f
        private const val V_D_GAIN = 0.3f
        private const val MAX_V_MPS = 0.3f
        private const val MIN_V_MPS = 0.08f   // floor to beat the FC velocity filter
    }
}
