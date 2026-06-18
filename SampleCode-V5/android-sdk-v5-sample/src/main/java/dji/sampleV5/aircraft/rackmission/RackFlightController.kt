package dji.sampleV5.aircraft.rackmission

import android.util.Log
import dji.sampleV5.aircraft.rackscan.CameraIntrinsics
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
import org.opencv.calib3d.Calib3d
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.opencv.core.MatOfPoint2f
import org.opencv.core.MatOfPoint3f
import org.opencv.core.Point
import org.opencv.core.Point3
import org.opencv.objdetect.ArucoDetector
import org.opencv.objdetect.DetectorParameters
import org.opencv.objdetect.Objdetect
import java.util.Timer
import java.util.TimerTask
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.abs
import kotlin.math.sqrt

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

    /** ALIGN aligns one axis at a time, in this order, so each command is isolated
     *  and the multi-axis coupling (which looked like circular drift) is gone. */
    enum class AlignPhase { VERTICAL, LATERAL, DISTANCE, DONE }

    @Volatile var sweepSpeedMps: Float = DEFAULT_SWEEP_SPEED_MPS
    @Volatile private var state: FlightState = FlightState.IDLE
    @Volatile private var vsEnabled = false
    @Volatile private var climbStartAltitudeM = 0.0
    @Volatile private var climbDistanceM = 0f

    // ── ArUco align state ──
    @Volatile private var arucoDetector: ArucoDetector? = null
    @Volatile private var arucoReady = false

    // ── solvePnP pose estimation (camera-calibrated metric positioning) ──
    // Camera matrix is rebuilt only when the frame resolution changes; object
    // points only when the marker size changes. distCoeffs are constant.
    private val distCoeffs: MatOfDouble by lazy { CameraIntrinsics.distCoeffs() }
    private var camMatrix: Mat? = null
    private var camMatrixW = 0
    private var camMatrixH = 0
    private var objPoints: MatOfPoint3f? = null
    private var objPointsSizeM = -1f
    @Volatile private var targetMarkerId = 0
    // ALIGN runs all three axes in VELOCITY mode (m/s) so ground speed is bounded
    // and 0 = the FC holds zero velocity (brakes) — it settles at the standoff
    // instead of coasting into the marker the way ANGLE-mode tilt did.
    //
    // AXIS MAPPING (this airframe): the body roll axis moves the drone fore/aft
    // and the body pitch axis moves it sideways — i.e. swapped from the usual
    // convention (the sweep code already drives left/right via the pitch axis).
    // So standoff/depth control is sent on ROLL and lateral centering on PITCH.
    @Volatile private var targetRollVel = 0f      // roll axis → standoff / depth velocity (m/s)
    @Volatile private var targetVz = 0f           // elevation centering (vertical velocity, m/s)
    @Volatile private var targetPitchVel = 0f     // pitch axis → lateral centering velocity (m/s)
    @Volatile private var prevOffsetX = 0f
    @Volatile private var prevOffsetY = 0f
    @Volatile private var prevDistErr = 0f
    @Volatile private var inHoverLock = true      // true when not actively correcting laterally (telemetry/HUD)
    // Sequential-align sub-state machine.
    @Volatile private var alignPhase = AlignPhase.VERTICAL
    @Volatile private var phaseInZoneSinceMs = 0L // when the active axis first entered its deadzone (0 = not in zone)

    // Metric distance config (set from the dashboard).
    @Volatile private var markerSizeM = 0.10f      // physical ArUco side length (m)
    @Volatile private var targetDistanceM = 1.0f   // desired standoff (m)
    @Volatile private var cameraHfovDeg = CAMERA_HFOV_DEG  // horizontal FOV (°), calibratable
    // distance(m) = markerSizeM / (2·tan(HFOV/2) · sizeNorm). Frame width cancels,
    // so only the marker's real size + camera FOV are needed (no per-res calibration).
    @Volatile private var distK = 1.0 / (2.0 * Math.tan(Math.toRadians(CAMERA_HFOV_DEG / 2.0)))
    @Volatile private var centered = false        // true only when centered on BOTH axes
    @Volatile private var wasDetected = false     // edge-detect so we log acquisition once, not per frame

    private var pumpTimer: Timer? = null
    private val pumpTicks = AtomicLong(0)

    fun start() {
        if (pumpTimer != null) return
        telemetry.sweepSpeedMps = sweepSpeedMps
        telemetry.arucoSizeM = markerSizeM
        telemetry.standoffM = targetDistanceM
        telemetry.cameraHfovDeg = cameraHfovDeg.toFloat()
        pumpTimer = Timer("RackMissionPump", true).also {
            it.scheduleAtFixedRate(object : TimerTask() {
                override fun run() = pumpTick()
            }, 0L, COMMAND_INTERVAL_MS)
        }
    }

    fun shutdown() {
        pumpTimer?.cancel(); pumpTimer = null
        if (vsEnabled) disableVS()
        camMatrix?.release(); camMatrix = null
        objPoints?.release(); objPoints = null
    }

    fun setSweepSpeed(mps: Float) {
        sweepSpeedMps = mps.coerceIn(MIN_SWEEP_MPS, MAX_SWEEP_MPS)
        telemetry.sweepSpeedMps = sweepSpeedMps
        logs.i(TAG, "Sweep speed set to ${"%.2f".format(sweepSpeedMps)} m/s")
    }

    /** Physical ArUco marker side length (m) — needed to convert apparent size to metres. */
    fun setArucoSize(m: Float) {
        markerSizeM = m.coerceIn(0.02f, 2.0f)
        telemetry.arucoSizeM = markerSizeM
        logs.i(TAG, "ArUco marker size set to ${"%.3f".format(markerSizeM)} m")
    }

    /** Target standoff distance (m) the aligner holds from the marker. */
    fun setStandoff(m: Float) {
        targetDistanceM = m.coerceIn(0.3f, 10.0f)
        telemetry.standoffM = targetDistanceM
        logs.i(TAG, "Target standoff set to ${"%.2f".format(targetDistanceM)} m")
    }

    /** Camera horizontal FOV (°) — calibrates the apparent-size→metres conversion. */
    fun setCameraHfov(deg: Float) {
        cameraHfovDeg = deg.toDouble().coerceIn(20.0, 160.0)
        distK = 1.0 / (2.0 * Math.tan(Math.toRadians(cameraHfovDeg / 2.0)))
        telemetry.cameraHfovDeg = cameraHfovDeg.toFloat()
        logs.i(TAG, "Camera HFOV set to ${"%.1f".format(cameraHfovDeg)}°")
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
        if (ensureDetector() == null) { logs.e(TAG, "OpenCV init failed — cannot align"); return }
        prevOffsetX = 0f; prevOffsetY = 0f; prevDistErr = 0f
        inHoverLock = true
        centered = false; targetRollVel = 0f; targetVz = 0f; targetPitchVel = 0f; wasDetected = false
        alignPhase = AlignPhase.VERTICAL; phaseInZoneSinceMs = 0L
        telemetry.alignPhase = alignPhase.name
        telemetry.currentTargetId = markerId; telemetry.isTracking = true
        state = FlightState.ALIGN
        logs.i(TAG, "Aligning to ArUco marker $markerId")
        logs.i(TAG, "[ALIGN] ▸ start → ${phaseLabel(AlignPhase.VERTICAL)}")
    }

    override fun isCentered(): Boolean = state == FlightState.ALIGN && centered

    override fun missionStopAndHover() {
        centered = false; targetRollVel = 0f; targetVz = 0f; targetPitchVel = 0f; wasDetected = false
        phaseInZoneSinceMs = 0L; telemetry.alignPhase = "—"
        telemetry.isTracking = false; telemetry.detected = false; telemetry.alignCentered = false
        state = if (vsEnabled) FlightState.HOVER else FlightState.IDLE
    }

    override fun missionLand() { land() }

    // ── ArUco frame processing (called from the ViewModel's encoder thread) ──

    /**
     * Detect ArUco markers in [nv21], publish distance/offset telemetry, and —
     * only while an ALIGN step is active — drive the centering command +
     * [centered] flag.
     *
     * Detection ALSO runs outside ALIGN (called every frame the dashboard is
     * watching) so the operator can eyeball the measured distance live before
     * committing to a mission. In that "measure" mode it reports the first
     * marker the detector returns and issues no flight commands.
     */
    fun processAlignFrame(nv21: ByteArray, width: Int, height: Int) {
        val det = ensureDetector() ?: return
        val aligning = state == FlightState.ALIGN
        // ArUco needs only the grayscale Y plane; NV21's first w*h bytes are exactly that.
        val yuvMat = Mat(height + height / 2, width, CvType.CV_8UC1)
        yuvMat.put(0, 0, nv21)
        val gray = yuvMat.submat(0, height, 0, width)
        val corners = mutableListOf<Mat>()
        val ids = Mat()
        try {
            det.detectMarkers(gray, corners, ids)
            var detected = false
            var markerId = -1
            var offsetX = 0f
            var offsetY = 0f
            var sizeNorm = 0f
            var cornerPx: FloatArray? = null   // chosen marker corners, for solvePnP
            var cornerPy: FloatArray? = null
            if (ids.rows() > 0) {
                // While aligning, only the configured target may drive control.
                // While measuring, take the first marker the detector returns.
                var idx = -1
                if (aligning) {
                    for (i in 0 until ids.rows()) if (ids[i, 0][0].toInt() == targetMarkerId) { idx = i; break }
                } else {
                    idx = 0
                }
                if (idx >= 0) {
                    detected = true
                    markerId = ids[idx, 0][0].toInt()
                    val mc = corners[idx]
                    val px = FloatArray(4); val py = FloatArray(4)
                    for (j in 0 until 4) { px[j] = mc[0, j][0].toFloat(); py[j] = mc[0, j][1].toFloat() }
                    cornerPx = px; cornerPy = py
                    val cx = (px[0] + px[1] + px[2] + px[3]) / 4f
                    val cy = (py[0] + py[1] + py[2] + py[3]) / 4f
                    offsetX = (cx - width / 2f) / (width / 2f)
                    offsetY = (cy - height / 2f) / (height / 2f)   // >0 = marker below frame centre
                    // Apparent size = sqrt(quad area) (shoelace), normalised to frame width.
                    var area2 = 0.0
                    for (j in 0 until 4) { val k = (j + 1) % 4; area2 += px[j] * py[k] - px[k] * py[j] }
                    sizeNorm = (sqrt(abs(area2) / 2.0).toFloat()) / width
                }
            }
            if (detected) {
                // solvePnP gives the true metric standoff (Z along the optical
                // axis) — tilt-robust, unlike the apparent-size heuristic. Fall
                // back to the heuristic only if pose estimation fails.
                val pose = cornerPx?.let { estimatePose(it, cornerPy!!, width, height) }
                val distM = pose?.get(2)?.toFloat() ?: distanceFromSize(sizeNorm)  // metres
                telemetry.distanceM = distM
                telemetry.poseValid = pose != null
                telemetry.posX = pose?.get(0)?.toFloat() ?: 0f
                telemetry.posY = pose?.get(1)?.toFloat() ?: 0f
                telemetry.posZ = pose?.get(2)?.toFloat() ?: 0f
                if (aligning) {
                    // Sequential alignment: only the active phase's axis is driven;
                    // the others are held at 0. Advance once the active axis holds
                    // inside its deadzone for PHASE_SETTLE_MS. Isolating one axis at
                    // a time removes the multi-axis coupling that looked circular.
                    targetVz = 0f; targetRollVel = 0f; targetPitchVel = 0f
                    val now = System.currentTimeMillis()
                    when (alignPhase) {
                        AlignPhase.VERTICAL -> {
                            targetVz = computeVertical(offsetY)
                            if (settled(abs(offsetY) < DEAD_ZONE_Y, now)) {
                                prevOffsetX = offsetX  // seed D-term for the next axis
                                setAlignPhase(AlignPhase.LATERAL, "elevation centered (y=${"%.2f".format(offsetY)})")
                            }
                        }
                        AlignPhase.LATERAL -> {
                            // Lateral centering drives the PITCH axis (see AXIS MAPPING).
                            targetPitchVel = computeLateralVel(offsetX)
                            if (settled(abs(offsetX) < DEAD_ZONE, now)) {
                                prevDistErr = distM - targetDistanceM
                                setAlignPhase(AlignPhase.DISTANCE, "lateral centered (x=${"%.2f".format(offsetX)})")
                            }
                        }
                        AlignPhase.DISTANCE -> {
                            // Standoff/depth drives the ROLL axis (see AXIS MAPPING).
                            targetRollVel = computeApproachVel(distM)
                            if (settled(abs(distM - targetDistanceM) < DIST_DEADZONE_M, now)) {
                                setAlignPhase(AlignPhase.DONE, "distance held (${"%.2f".format(distM)} m)")
                            }
                        }
                        AlignPhase.DONE -> { /* aligned — hold all axes at 0 */ }
                    }
                    centered = alignPhase == AlignPhase.DONE
                }
            } else {
                telemetry.distanceM = 0f
                telemetry.poseValid = false; telemetry.posX = 0f; telemetry.posY = 0f; telemetry.posZ = 0f
                if (aligning) {
                    // Marker lost mid-align → command zero velocity on all axes so
                    // the FC brakes and holds. Keep the phase, but make the active
                    // axis re-settle on reacquire (don't credit time spent blind).
                    targetRollVel = 0f; targetVz = 0f; targetPitchVel = 0f; centered = false
                    prevOffsetX = 0f; prevOffsetY = 0f; prevDistErr = 0f; inHoverLock = true
                    phaseInZoneSinceMs = 0L
                }
            }
            // Acquisition/loss logging only matters for an active align (else it
            // would spam as markers drift in and out during free measuring).
            if (aligning && detected != wasDetected) {
                if (detected) logs.i(TAG, "[DETECTED]: Aruco ID: $targetMarkerId detected.")
                else          logs.w(TAG, "[LOST]: Aruco ID: $targetMarkerId out of view.")
                wasDetected = detected
            }
            telemetry.detected = detected
            telemetry.detectedId = markerId
            telemetry.offsetX = offsetX
            telemetry.offsetY = offsetY
            telemetry.markerSize = sizeNorm
            telemetry.alignCentered = aligning && centered
        } catch (t: Throwable) {
            Log.w(TAG, "align detect: ${t.message}")
        } finally {
            gray.release(); ids.release(); corners.forEach { it.release() }; yuvMat.release()
        }
    }

    /** True once [inZone] has held continuously for [PHASE_SETTLE_MS]; resets the
     *  timer the moment the axis leaves its deadzone (so a brief dip doesn't count). */
    private fun settled(inZone: Boolean, nowMs: Long): Boolean {
        if (!inZone) { phaseInZoneSinceMs = 0L; return false }
        if (phaseInZoneSinceMs == 0L) phaseInZoneSinceMs = nowMs
        return nowMs - phaseInZoneSinceMs >= PHASE_SETTLE_MS
    }

    /** Advance the sequential-align state machine, mirror it to telemetry, and log. */
    private fun setAlignPhase(next: AlignPhase, reason: String) {
        alignPhase = next
        phaseInZoneSinceMs = 0L
        telemetry.alignPhase = next.name
        logs.i(TAG, "[ALIGN] ▸ $reason → ${phaseLabel(next)}")
    }

    private fun phaseLabel(p: AlignPhase): String = when (p) {
        AlignPhase.VERTICAL -> "elevation (up/down)"
        AlignPhase.LATERAL  -> "lateral (left/right)"
        AlignPhase.DISTANCE -> "distance (forward/back)"
        AlignPhase.DONE     -> "DONE (centered)"
    }

    /** Lazily build the ArUco detector so detection (for live distance preview)
     *  can run before any ALIGN step. Returns null only if OpenCV won't load. */
    private fun ensureDetector(): ArucoDetector? {
        arucoDetector?.let { return it }
        if (!arucoReady) {
            if (!OpenCVLoader.initLocal()) { Log.w(TAG, "OpenCV init failed — no ArUco"); return null }
            arucoReady = true
        }
        return ArucoDetector(Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50), DetectorParameters())
            .also { arucoDetector = it; logs.i(TAG, "ArUco detector ready (DICT_4X4_50)") }
    }

    /** PD → lateral (body-right) velocity in m/s to centre the marker horizontally.
     *  Marker right of centre (offsetX > 0) → fly right (+). A small floor beats
     *  the FC velocity filter on the last bit of centering; 0 = hold (FC brakes). */
    private fun computeLateralVel(offsetX: Float): Float {
        if (abs(offsetX) < DEAD_ZONE) { prevOffsetX = offsetX; inHoverLock = true; return 0f }
        inHoverLock = false
        val p = LAT_P_GAIN * offsetX
        val d = LAT_D_GAIN * (offsetX - prevOffsetX)
        prevOffsetX = offsetX
        var v = p + d
        if (v != 0f && abs(v) < MIN_LAT_MPS) v = if (v > 0) MIN_LAT_MPS else -MIN_LAT_MPS
        return v.coerceIn(-MAX_LAT_MPS, MAX_LAT_MPS)
    }

    /** Estimated drone↔marker distance in metres from the marker's apparent size.
     *  distance = markerSizeM / (2·tan(HFOV/2) · sizeNorm). Fallback only — used
     *  when [estimatePose] fails; solvePnP is the primary path. */
    private fun distanceFromSize(sizeNorm: Float): Float =
        if (sizeNorm <= 1e-4f) 99f else (markerSizeM * distK / sizeNorm).toFloat()

    /**
     * Calibrated marker pose via solvePnP. Returns the marker centre in the
     * CAMERA frame as [X, Y, Z] metres (X = right, Y = down, Z = forward along
     * the optical axis), or null if estimation fails. [SOLVEPNP_IPPE_SQUARE] is
     * purpose-built for a planar square marker given its four corners.
     */
    private fun estimatePose(px: FloatArray, py: FloatArray, width: Int, height: Int): DoubleArray? {
        val img = MatOfPoint2f(
            Point(px[0].toDouble(), py[0].toDouble()),
            Point(px[1].toDouble(), py[1].toDouble()),
            Point(px[2].toDouble(), py[2].toDouble()),
            Point(px[3].toDouble(), py[3].toDouble()),
        )
        val rvec = Mat(); val tvec = Mat()
        return try {
            val ok = Calib3d.solvePnP(
                objectPointsFor(markerSizeM), img,
                cameraMatrixFor(width, height), distCoeffs,
                rvec, tvec, false, Calib3d.SOLVEPNP_IPPE_SQUARE,
            )
            if (ok) doubleArrayOf(tvec[0, 0][0], tvec[1, 0][0], tvec[2, 0][0]) else null
        } catch (t: Throwable) {
            Log.w(TAG, "solvePnP: ${t.message}"); null
        } finally {
            img.release(); rvec.release(); tvec.release()
        }
    }

    /** Camera matrix for the live frame size, rescaled from the calibration
     *  resolution and cached until the resolution changes. */
    private fun cameraMatrixFor(width: Int, height: Int): Mat {
        camMatrix?.let { if (width == camMatrixW && height == camMatrixH) return it; it.release() }
        return CameraIntrinsics.cameraMatrix(width, height).also {
            camMatrix = it; camMatrixW = width; camMatrixH = height
        }
    }

    /** Marker corner object points (metres) in the marker's own frame, ordered
     *  to match ArUco's detected-corner order (TL, TR, BR, BL). Cached per size. */
    private fun objectPointsFor(sizeM: Float): MatOfPoint3f {
        objPoints?.let { if (sizeM == objPointsSizeM) return it; it.release() }
        val h = sizeM / 2.0
        return MatOfPoint3f(
            Point3(-h,  h, 0.0),
            Point3( h,  h, 0.0),
            Point3( h, -h, 0.0),
            Point3(-h, -h, 0.0),
        ).also { objPoints = it; objPointsSizeM = sizeM }
    }

    /** PD (metres) → forward/back velocity in m/s to hold the standoff. Too far
     *  (distance > target) → approach the marker; too close → back off. Hard-capped
     *  to a deliberately low speed, and 0 = hold (the FC brakes), so the drone
     *  settles at the standoff instead of coasting into the marker. Forward is
     *  toward the rack and obstacle-avoidance is bypassed, so this is kept slow.
     *
     *  Sign: on this airframe +roll velocity drives the drone toward the marker
     *  (verified in flight), so the PD output is used as-is. */
    private fun computeApproachVel(distM: Float): Float {
        val err = distM - targetDistanceM            // >0 = too far → must approach
        if (abs(err) < DIST_DEADZONE_M) { prevDistErr = err; return 0f }
        var v = APPROACH_KP_MPS_PER_M * err + APPROACH_KD * (err - prevDistErr)
        prevDistErr = err
        if (v != 0f && abs(v) < MIN_APPROACH_MPS) v = if (v > 0) MIN_APPROACH_MPS else -MIN_APPROACH_MPS
        return v.coerceIn(-MAX_APPROACH_MPS, MAX_APPROACH_MPS)
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
                if (sweeping || s == FlightState.ALIGN) {
                    // VELOCITY mode for both axes: ground speed is bounded and 0 =
                    // the FC holds zero velocity (brakes). ALIGN settles at the
                    // standoff instead of coasting forward as ANGLE-mode tilt did.
                    rollPitchControlMode = RollPitchControlMode.VELOCITY
                    if (sweeping) {
                        val dir = if (s == FlightState.SWEEP_RIGHT) 1.0 else -1.0
                        pitch = sweepSpeedMps.toDouble() * dir
                        roll  = 0.0
                    } else {
                        roll  = targetRollVel.toDouble()   // standoff / depth (m/s) — see AXIS MAPPING
                        pitch = targetPitchVel.toDouble()  // lateral centering (m/s) — see AXIS MAPPING
                    }
                } else {
                    // HOVER: hold level.
                    rollPitchControlMode = RollPitchControlMode.ANGLE
                    roll  = 0.0
                    pitch = 0.0
                }
            }
            VirtualStickManager.getInstance().sendVirtualStickAdvancedParam(param)
            pumpTicks.incrementAndGet()

            telemetry.stateName            = s.name
            telemetry.rollPitchControlMode = if (param.rollPitchControlMode == RollPitchControlMode.VELOCITY) "VELOCITY" else "ANGLE"
            telemetry.verticalControlMode  = verticalMode.name
            telemetry.rollDeg              = param.roll       // ALIGN: standoff/depth velocity (m/s)
            telemetry.pitchMps             = param.pitch      // ALIGN: lateral velocity (m/s)
            telemetry.verticalThrottleMps  = param.verticalThrottle
            telemetry.pumpTicks            = pumpTicks.get()
            telemetry.sweepSpeedMps        = sweepSpeedMps
            telemetry.inHoverLock          = inHoverLock
            telemetry.inKick               = false
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
        // Sequential align: how long the active axis must hold inside its deadzone
        // before advancing to the next axis (debounces noise / brief overshoot).
        private const val PHASE_SETTLE_MS = 500L
        // ArUco centering — all axes VELOCITY-mode m/s, gentle for indoor flight.
        // DEAD_ZONE = "close enough" tolerance: bigger = stops correcting sooner
        // (less fidgeting), smaller = tries harder to perfectly centre.
        private const val DEAD_ZONE = 0.10f
        // Lateral (roll) centering — body-right velocity from normalised offsetX.
        private const val LAT_P_GAIN = 0.40f
        private const val LAT_D_GAIN = 0.20f
        private const val MAX_LAT_MPS = 0.25f
        private const val MIN_LAT_MPS = 0.06f   // floor to beat the FC velocity filter
        // Vertical (elevation) centering — VELOCITY-mode m/s, gentle for indoor.
        private const val DEAD_ZONE_Y = 0.12f
        private const val V_P_GAIN = 0.6f
        private const val V_D_GAIN = 0.3f
        private const val MAX_V_MPS = 0.3f
        private const val MIN_V_MPS = 0.08f   // floor to beat the FC velocity filter
        // Distance / standoff hold — forward/back VELOCITY (m/s), metric. Speed is
        // capped deliberately low: forward is toward the rack and obstacle-avoidance
        // is bypassed, so this is the safety-critical axis. 0 = hold (FC brakes).
        private const val CAMERA_HFOV_DEG = 82.0       // fallback heuristic only (solvePnP is primary)
        private const val DIST_DEADZONE_M = 0.10f      // ±10 cm "close enough"
        private const val APPROACH_KP_MPS_PER_M = 0.30f // 1 m of error → 0.30 m/s (capped well below)
        private const val APPROACH_KD = 0.10f
        private const val MIN_APPROACH_MPS = 0.05f     // floor so the last bit still closes
        private const val MAX_APPROACH_MPS = 0.15f     // HARD speed cap — ~15 cm/s, intentionally slow
    }
}
