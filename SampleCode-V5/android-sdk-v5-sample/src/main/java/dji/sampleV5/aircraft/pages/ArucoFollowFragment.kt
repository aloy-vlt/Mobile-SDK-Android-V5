package dji.sampleV5.aircraft.pages

import android.graphics.Bitmap
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.SeekBar
import android.widget.TextView
import android.widget.ImageView
import android.widget.EditText
import androidx.fragment.app.activityViewModels
import dji.sampleV5.aircraft.R
import dji.sampleV5.aircraft.models.BasicAircraftControlVM
import dji.sampleV5.aircraft.rackscan.MissionExecutor
import dji.sampleV5.aircraft.rackscan.MissionStep
import dji.sampleV5.aircraft.rackscan.RackScanDashboardServer
import dji.sampleV5.aircraft.rackscan.RackScanLogBuffer
import dji.sampleV5.aircraft.rackscan.RackScanTelemetry
import dji.sampleV5.aircraft.models.VirtualStickVM
import dji.sampleV5.aircraft.util.ToastUtils
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.et.create
import dji.v5.et.get
import dji.v5.et.listen
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.perception.PerceptionManager
import dji.v5.manager.aircraft.perception.data.ObstacleAvoidanceType
import dji.v5.manager.aircraft.perception.data.ObstacleData
import dji.v5.manager.aircraft.perception.data.PerceptionInfo
import dji.v5.manager.aircraft.perception.listener.ObstacleDataListener
import dji.v5.manager.aircraft.perception.listener.PerceptionInformationListener
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import org.opencv.android.OpenCVLoader
import org.opencv.android.Utils
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import org.opencv.objdetect.ArucoDetector
import org.opencv.objdetect.DetectorParameters
import org.opencv.objdetect.Objdetect
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong
import java.util.Timer
import java.util.TimerTask
import kotlin.math.abs

class ArucoFollowFragment : DJIFragment() {

    companion object {
        private const val TAG = "RackScanner"
        private const val DEAD_ZONE        = 0.05f
        private const val CENTER_THRESHOLD = 0.08f
        private const val SAFETY_TIMEOUT_MS   = 5000L
        private const val COMMAND_INTERVAL_MS = 50L   // 20 Hz

        // All movement uses ANGLE mode (BODY frame).
        // ANGLE mode is the only mode that reliably bypasses the FC's
        // noise / velocity-tracker filter on the Mini 4 Pro.
        private const val MAX_ROLL_ANGLE_DEG = 15.0f
        private const val MAX_ROLL_RATIO     = 0.6f

        // Kickstart — boosts a too-small angle command above the FC hover-filter
        // for KICK_DURATION_MS on the first non-zero output after a hover-lock.
        // Same mechanism the LAN Dashboard's aruco-follow uses: any requested
        // angle whose |ratio| ≥ KICK_RATIO passes through unchanged (no slam),
        // anything below is floored to ±KICK_RATIO so the FC can't filter it out.
        // Applies uniformly to centering, fine-tuning, AND sweep.
        private const val KICK_RATIO       = 0.25f   // min ratio (≈3.75° tilt)
        private const val KICK_DURATION_MS = 300L

        private const val SWEEP_CAPTURE_ZONE   = 0.55f   // normalised offset
        private const val CENTERING_MISS_TOL   = 5       // frames before hoverStick
        private const val OBSTACLE_WARN_M      = 1.5f

        // Auto-descend after takeoff: drone hovers at ~1.2m default, we drop
        // it to TARGET_TAKEOFF_HEIGHT for close-range rack scanning.
        private const val TARGET_TAKEOFF_HEIGHT_M       = 0.50f
        private const val TAKEOFF_DESCEND_SPEED_MPS     = 0.20f
        private const val TAKEOFF_STABILIZE_MS          = 5000L
        private const val TAKEOFF_DESCEND_TIMEOUT_MS    = 8000L
        private const val ALT_TOLERANCE_M               = 0.05f

        // FC altitude cap raised after takeoff, restored on exit.
        private const val TARGET_HEIGHT_LIMIT_M         = 10

        // Minimum duration in CLIMBING_UP/DOWN before the capture-zone check
        // is allowed to fire. Without this, if currentTargetId happens to be
        // a marker already in frame (e.g. the same-level partner), the climb
        // ends on frame 1 and altitude barely changes.
        private const val MIN_CLIMB_DURATION_MS         = 800L

    }

    enum class ScanState {
        IDLE, CENTERING, CENTERED,
        SWEEPING_LEFT, SWEEPING_RIGHT,
        CLIMBING_UP, CLIMBING_DOWN, FINE_TUNING, COMPLETE,
        TAKEOFF_DESCENDING
    }

    private val basicAircraftControlVM: BasicAircraftControlVM by activityViewModels()
    private val virtualStickVM: VirtualStickVM by activityViewModels()

    // UI refs
    private lateinit var tvState: TextView
    private lateinit var tvMarkerInfo: TextView
    private lateinit var tvSafety: TextView
    private lateinit var tvStickDebug: TextView
    private lateinit var imgPreview: ImageView
    private lateinit var etLevels: EditText
    private lateinit var tvConfigInfo: TextView
    private lateinit var seekPGain: SeekBar
    private lateinit var seekDGain: SeekBar
    private lateinit var seekSweepSpeed: SeekBar
    private lateinit var seekClimbSpeed: SeekBar
    private lateinit var btnTakeOff: Button
    private lateinit var btnEnableVS: Button
    private lateinit var btnLand: Button
    private lateinit var btnStartScan: Button
    private lateinit var btnGoLeft: Button
    private lateinit var btnGoRight: Button
    private lateinit var btnGoUp: Button
    private lateinit var btnGoDown: Button
    private lateinit var btnStop: Button
    private lateinit var btnOaBrake: Button
    private lateinit var btnOaBypass: Button
    private lateinit var btnOaClose: Button

    // Config
    private var numLevels    = 4
    private var totalMarkers = 8

    // State machine
    @Volatile private var state = ScanState.IDLE
    private var currentTargetId = 0
    private var currentLevel    = 0
    private var isVSEnabled     = false

    // PD controller (only used for CENTERING / FINE_TUNING — see pump)
    private var pGain      = 0.12f
    private var dGain      = 0.08f
    private var prevOffsetX = 0f

    // Sweep cruise speed in m/s. Sweep uses VELOCITY mode (not ANGLE) because
    // ANGLE with a steady-state attitude command fights the FC's position-hold
    // outer loop; VELOCITY tells the FC to *travel* and is designed for cruise.
    // Kept at minimum-safe values for rack scanning — close to obstacles means
    // anything above ~0.3 m/s leaves little time to stop. Mini 4 Pro's velocity
    // tracker silently ignores commands below ~0.10 m/s, so that's the floor.
    private var sweepSpeedMps = 0.15f   // 0.10–0.40 m/s in 0.05 steps

    // Climb is now POSITION-mode: we capture altitude at climb-start and send
    // an absolute target altitude ±climbDistanceM. The FC's position controller
    // is a separate loop from the velocity tracker that swallowed our prior
    // m/s commands. The "slider" is now distance per press, not speed.
    // Default matches the user's rack: 50 cm between levels (and 50 cm between
    // same-level markers, but that's handled by sweep speed, not this value).
    private var climbDistanceM = 0.5f   // 0.50–2.00 m in 0.25 m steps
    // Kept around for telemetry display (and in case a future change goes back
    // to VELOCITY mode) — currently unused by the pump.
    private var climbSpeedMps = 0.30f
    // Captured at startClimb so the absolute target is stable for the duration
    // of the climb (otherwise the moving currentAltitudeM would shift the goal
    // post and the drone would never settle).
    @Volatile private var climbStartAltitudeM: Double = 0.0

    // Climb start time (for safety-timeout watchdog only; no fixed duration)
    private var climbStartTime = 0L

    // Live altitude (m), updated by KeyAltitude listener
    @Volatile private var currentAltitudeM: Double = 0.0
    private var takeoffDescendStartMs: Long = 0L

    // FC max-altitude cap. The fragment raises it to TARGET_HEIGHT_LIMIT_M
    // for the duration of the scan so UP isn't blocked at the user's default
    // safety cap (often 1–2m in Pilot for indoor flying), then restores the
    // prior value in onDestroyView. `originalHeightLimit = null` means we
    // haven't read/changed it yet (so don't try to restore).
    private var originalHeightLimit: Int? = null

    // Dashboard server (interactive telemetry+mission UI at http://<phone>:8082)
    private val telemetry = RackScanTelemetry()
    private val logBuffer = RackScanLogBuffer()
    private var dashboardServer: RackScanDashboardServer? = null
    private val missionExecutor: MissionExecutor by lazy {
        MissionExecutor(missionHost, telemetry, logBuffer)
    }

    // Sweep leg tracking
    private var sweepTargetAdvanced  = false
    private var centeringMissFrames  = 0

    // Safety
    @Volatile private var lastMarkerSeenTime = 0L
    private var movementStartTime    = 0L
    private var safetyStopTriggered  = false

    // ── PUMP TARGETS (written by frame thread, read by pump thread) ──
    // targetRollDeg drives ANGLE-mode lateral correction during CENTERING /
    // FINE_TUNING. Vertical throttle is derived from `state` directly in the
    // pump (CLIMBING_UP / CLIMBING_DOWN / TAKEOFF_DESCENDING), so there's no
    // separate throttle field to keep in sync.
    @Volatile private var targetRollDeg: Float = 0f

    // Kickstart state (shared by centering, fine-tuning, and sweep)
    @Volatile private var inHoverLock: Boolean = true
    @Volatile private var kickEndsAtMs: Long   = 0L

    // Pump diagnostic counter — visible on screen so operator confirms pump is alive
    private val pumpTicks = AtomicLong(0L)

    // Obstacle detection
    @Volatile private var nearestObstacleM: Float = -1f
    private val obstacleListener = ObstacleDataListener { data: ObstacleData? ->
        nearestObstacleM = try {
            when (val h = data?.horizontalObstacleDistance) {
                null           -> -1f
                is Number      -> h.toFloat().takeIf { it > 0f } ?: -1f
                is DoubleArray -> h.filter { it > 0 }.minOrNull()?.toFloat() ?: -1f
                is FloatArray  -> h.filter { it > 0f }.minOrNull() ?: -1f
                is List<*>     -> h.filterIsInstance<Number>().map { it.toFloat() }
                                   .filter { it > 0f }.minOrNull() ?: -1f
                else           -> -1f
            }
        } catch (_: Throwable) { -1f }
    }

    // Current OA mode — null until the first PerceptionInfo arrives. UI highlights match this.
    @Volatile private var currentOaType: ObstacleAvoidanceType? = null
    private val perceptionInfoListener = PerceptionInformationListener { info: PerceptionInfo? ->
        val t = info?.obstacleAvoidanceType ?: return@PerceptionInformationListener
        if (t != currentOaType) {
            currentOaType = t
            mainHandler.post { refreshOaButtons() }
        }
    }

    private var commandTimer: Timer? = null

    // OpenCV
    private lateinit var arucoDetector: ArucoDetector
    private val isTracking   = AtomicBoolean(false)
    private val isProcessing = AtomicBoolean(false)
    private lateinit var processingThread: HandlerThread
    private lateinit var processingHandler: Handler

    private val frameListener = object : ICameraStreamManager.CameraFrameListener {
        override fun onFrame(data: ByteArray, off: Int, len: Int, w: Int, h: Int,
                             fmt: ICameraStreamManager.FrameFormat) {
            if (!isTracking.get() || isProcessing.get()) return
            val copy = ByteArray(len).also { System.arraycopy(data, off, it, 0, len) }
            processingHandler.post { processFrame(copy, w, h) }
        }
    }

    override fun onCreateView(i: LayoutInflater, c: ViewGroup?, s: Bundle?): View? =
        i.inflate(R.layout.frag_aruco_follow, c, false)

    override fun onViewCreated(view: View, saved: Bundle?) {
        super.onViewCreated(view, saved)
        if (!OpenCVLoader.initLocal()) { ToastUtils.showToast("OpenCV failed"); return }
        arucoDetector = ArucoDetector(
            Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50), DetectorParameters())
        processingThread = HandlerThread("RackScanner").also { it.start() }
        processingHandler = Handler(processingThread.looper)
        initUI(view)
        setupCameraStream()
        startCommandLoop()
        observeVirtualStickState()
        try { PerceptionManager.getInstance().addObstacleDataListener(obstacleListener) }
        catch (_: Throwable) {}
        try { PerceptionManager.getInstance().addPerceptionInformationListener(perceptionInfoListener) }
        catch (_: Throwable) {}
        // Dashboard: starts a tiny WS server on port 8082 streaming telemetry
        // and the log ring buffer. Browse to http://<phone-ip>:8082 to inspect.
        try {
            dashboardServer = RackScanDashboardServer(
                port = RackScanDashboardServer.DEFAULT_PORT,
                appContext = requireContext().applicationContext,
                telemetry = telemetry,
                logs = logBuffer,
            ).also {
                it.commandHandler = dashboardCommandHandler
                it.start()
            }
            logBuffer.i(TAG, "Dashboard server up at port ${RackScanDashboardServer.DEFAULT_PORT}")
        } catch (t: Throwable) {
            Log.w(TAG, "Dashboard server failed to start: ${t.message}")
        }
        try {
            FlightControllerKey.KeyAltitude.create().listen(this) { alt: Double? ->
                val a = alt ?: return@listen
                currentAltitudeM = a
                if (state == ScanState.TAKEOFF_DESCENDING && a <= TARGET_TAKEOFF_HEIGHT_M + ALT_TOLERANCE_M) {
                    mainHandler.post { onTakeoffDescentComplete() }
                }
            }
        } catch (_: Throwable) {}
        try {
            FlightControllerKey.KeyAircraftVelocity.create().listen(this) { v ->
                v?.let {
                    telemetry.velocityX = it.x
                    telemetry.velocityY = it.y
                    telemetry.velocityZ = it.z
                }
            }
        } catch (_: Throwable) {}
        try {
            FlightControllerKey.KeyAreMotorsOn.create().listen(this) { on ->
                if (on != null) telemetry.motorsOn = on
            }
        } catch (_: Throwable) {}
        // raiseHeightLimitForScan() runs on takeoff success — the read needs
        // a connected drone, which isn't guaranteed at view-created time.
    }

    // ── VS state observer: detect external kills, don't nuke scan state ──
    private fun observeVirtualStickState() {
        virtualStickVM.currentVirtualStickStateInfo.observe(viewLifecycleOwner) { info ->
            val s = info?.state ?: return@observe
            if (!s.isVirtualStickEnable && isVSEnabled) {
                isVSEnabled = false
                hoverStick()
                val reason = info.reason?.name ?: "unknown"
                mainHandler.post {
                    btnEnableVS.text = "ENABLE VS"
                    tvSafety.text = "VS off ($reason) — re-enable to continue"
                    tvSafety.visibility = View.VISIBLE
                    updateActionButtons()
                }
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    // 20 Hz STICK PUMP — HYBRID MODE
    //
    // Centering / Fine-tuning → ANGLE mode (BODY frame), roll = PD output.
    //
    // Sweeping LEFT / RIGHT → VELOCITY mode (BODY frame), pitch = ±sweepSpeed.
    //   (Roll-axis name → forward velocity in VELOCITY mode; pitch-axis name
    //   → lateral velocity. That's DJI's naming for V5.)
    //
    // Climbing UP / DOWN → ANGLE roll/pitch = 0, verticalThrottle = ±climbSpeed.
    //
    // Takeoff descent → ANGLE roll/pitch = 0, verticalThrottle = -descend speed
    //   until KeyAltitude listener drops below TARGET_TAKEOFF_HEIGHT.
    // ═══════════════════════════════════════════════════════════════════

    private fun startCommandLoop() {
        commandTimer = Timer("StickPump", true)
        commandTimer?.scheduleAtFixedRate(object : TimerTask() {
            override fun run() {
                val now = System.currentTimeMillis()
                if (!isVSEnabled || state == ScanState.IDLE || state == ScanState.COMPLETE) {
                    // Dashboard still wants the latest state/altitude/etc.
                    // even when we're not commanding the FC.
                    publishCommonTelemetry(now)
                    return
                }
                val isSweeping = state == ScanState.SWEEPING_LEFT ||
                                 state == ScanState.SWEEPING_RIGHT
                // Vertical control split: POSITION mode for code-driven climbs
                // (FC's position controller bypasses the velocity-tracker filter
                // that was swallowing our m/s commands); VELOCITY mode for the
                // takeoff auto-descent (which empirically works) and for hover
                // when no vertical motion is wanted.
                val useVerticalPosition = state == ScanState.CLIMBING_UP ||
                                          state == ScanState.CLIMBING_DOWN
                val verticalMode: VerticalControlMode =
                    if (useVerticalPosition) VerticalControlMode.POSITION
                    else VerticalControlMode.VELOCITY
                val verticalCommand: Double = when (state) {
                    ScanState.CLIMBING_UP        -> climbStartAltitudeM + climbDistanceM
                    ScanState.CLIMBING_DOWN      -> (climbStartAltitudeM - climbDistanceM).coerceAtLeast(0.3)
                    ScanState.TAKEOFF_DESCENDING -> -TAKEOFF_DESCEND_SPEED_MPS.toDouble()
                    else                         -> 0.0
                }

                try {
                    val param = VirtualStickFlightControlParam().apply {
                        rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                        yawControlMode            = YawControlMode.ANGULAR_VELOCITY
                        verticalControlMode       = verticalMode
                        yaw                       = 0.0
                        verticalThrottle          = verticalCommand
                        if (isSweeping) {
                            rollPitchControlMode = RollPitchControlMode.VELOCITY
                            val dir = if (state == ScanState.SWEEPING_RIGHT) 1.0 else -1.0
                            pitch = sweepSpeedMps.toDouble() * dir
                            roll  = 0.0
                        } else {
                            rollPitchControlMode = RollPitchControlMode.ANGLE
                            pitch = 0.0
                            roll = if (targetRollDeg == 0f) {
                                inHoverLock = true
                                0.0
                            } else {
                                if (inHoverLock) {
                                    inHoverLock = false
                                    kickEndsAtMs = now + KICK_DURATION_MS
                                }
                                val ratio = targetRollDeg / MAX_ROLL_ANGLE_DEG
                                val kicked = if (now < kickEndsAtMs && abs(ratio) < KICK_RATIO)
                                    (if (ratio > 0f) KICK_RATIO else -KICK_RATIO)
                                else ratio
                                (kicked * MAX_ROLL_ANGLE_DEG).toDouble()
                            }
                        }
                    }
                    VirtualStickManager.getInstance().sendVirtualStickAdvancedParam(param)
                    pumpTicks.incrementAndGet()

                    // Dashboard publish — capture pump output for inspection.
                    telemetry.rollPitchControlMode = if (isSweeping) "VELOCITY" else "ANGLE"
                    telemetry.verticalControlMode  = verticalMode.name
                    telemetry.rollDeg              = param.roll
                    telemetry.pitchMps             = param.pitch
                    telemetry.verticalThrottleMps  = param.verticalThrottle
                    telemetry.pumpTicks            = pumpTicks.get()
                    telemetry.inHoverLock          = inHoverLock
                    telemetry.inKick               = now < kickEndsAtMs
                    // Vertical kick no longer applies — POSITION mode doesn't
                    // pulse magnitudes. Pinned to false for UI clarity.
                    telemetry.inVerticalKick       = false
                    publishCommonTelemetry(now)
                } catch (t: Throwable) {
                    Log.w(TAG, "pump: ${t.message}")
                }

                // Safety watchdogs — sweep AND climb share the same 5s no-marker
                // rule. Skipped during mission execution: missions are time-
                // /distance-bounded by the executor and don't expect markers,
                // so the watchdog would always trip at 5s and prematurely halt
                // any step longer than that.
                if (!telemetry.missionRunning &&
                    (isSweeping ||
                     state == ScanState.CLIMBING_UP ||
                     state == ScanState.CLIMBING_DOWN)) {
                    if (now - lastMarkerSeenTime > SAFETY_TIMEOUT_MS &&
                        now - movementStartTime > SAFETY_TIMEOUT_MS) {
                        mainHandler.post { safetyStop("No marker ${SAFETY_TIMEOUT_MS / 1000}s") }
                    }
                }
                if (state == ScanState.TAKEOFF_DESCENDING &&
                    now - takeoffDescendStartMs > TAKEOFF_DESCEND_TIMEOUT_MS) {
                    mainHandler.post { onTakeoffDescentTimeout() }
                }
            }
        }, 0, COMMAND_INTERVAL_MS)
    }

    private fun stopCommandLoop() { commandTimer?.cancel(); commandTimer = null }

    /**
     * Capture fields that aren't owned by the pump itself — state machine,
     * gains/speeds, altitude, OA mode, obstacle distance. Called once per
     * pump tick so the dashboard reflects live state without each call site
     * needing to remember to update individual fields.
     */
    private fun publishCommonTelemetry(now: Long) {
        telemetry.stateName        = state.name
        telemetry.currentTargetId  = currentTargetId
        telemetry.currentLevel     = currentLevel
        telemetry.numLevels        = numLevels
        telemetry.totalMarkers     = totalMarkers
        telemetry.isTracking       = isTracking.get()
        telemetry.isVSEnabled      = isVSEnabled
        telemetry.pGain            = pGain
        telemetry.dGain            = dGain
        telemetry.sweepSpeedMps    = sweepSpeedMps
        telemetry.climbSpeedMps    = climbSpeedMps
        telemetry.altitudeM        = currentAltitudeM
        telemetry.oaMode           = currentOaType?.name ?: "UNKNOWN"
        telemetry.nearestObstacleM = nearestObstacleM
        telemetry.lastMarkerAgeSec =
            if (lastMarkerSeenTime > 0) (now - lastMarkerSeenTime) / 1000f else -1f
        telemetry.lastUpdateMs     = now
        // heightLimitM is set when the raise runs / restores; left alone here.
    }

    private fun hoverStick() {
        targetRollDeg = 0f
        inHoverLock   = true
    }

    // ════════════════════════════════════════════════════════════════════

    private fun initUI(view: View) {
        tvState      = view.findViewById(R.id.tv_state)
        tvMarkerInfo = view.findViewById(R.id.tv_marker_info)
        tvSafety     = view.findViewById(R.id.tv_safety)
        tvStickDebug = view.findViewById(R.id.tv_stick_debug)
        imgPreview   = view.findViewById(R.id.img_preview)
        tvConfigInfo = view.findViewById(R.id.tv_config_info)
        etLevels     = view.findViewById(R.id.et_levels)
        seekPGain       = view.findViewById(R.id.seek_pgain)
        seekDGain       = view.findViewById(R.id.seek_dgain)
        seekSweepSpeed  = view.findViewById(R.id.seek_sweep_speed)
        seekClimbSpeed  = view.findViewById(R.id.seek_climb_speed)
        btnTakeOff   = view.findViewById(R.id.btn_takeoff)
        btnEnableVS  = view.findViewById(R.id.btn_enable_vs)
        btnLand      = view.findViewById(R.id.btn_land)
        btnStartScan = view.findViewById(R.id.btn_start_scan)
        btnGoLeft    = view.findViewById(R.id.btn_go_left)
        btnGoRight   = view.findViewById(R.id.btn_go_right)
        btnGoUp      = view.findViewById(R.id.btn_go_up)
        btnGoDown    = view.findViewById(R.id.btn_go_down)
        btnStop      = view.findViewById(R.id.btn_stop)
        btnOaBrake   = view.findViewById(R.id.btn_oa_brake)
        btnOaBypass  = view.findViewById(R.id.btn_oa_bypass)
        btnOaClose   = view.findViewById(R.id.btn_oa_close)

        btnOaBrake .setOnClickListener { requestOaType(ObstacleAvoidanceType.BRAKE) }
        btnOaBypass.setOnClickListener { requestOaType(ObstacleAvoidanceType.BYPASS) }
        btnOaClose .setOnClickListener { requestOaType(ObstacleAvoidanceType.CLOSE) }
        refreshOaButtons()

        etLevels.setText(numLevels.toString())
        updateActionButtons()
        updateConfigText()

        btnTakeOff.setOnClickListener { doTakeoff() }
        btnEnableVS.setOnClickListener { doEnableVS() }
        btnLand.setOnClickListener    { doLand() }

        btnStartScan.setOnClickListener {
            val lv = etLevels.text.toString().toIntOrNull()
            if (lv == null || lv < 1 || lv > 12) { ToastUtils.showToast("Enter levels 1-12"); return@setOnClickListener }
            numLevels = lv; totalMarkers = numLevels * 2; startScan()
        }

        btnGoLeft.setOnClickListener  { if (state == ScanState.CENTERED) startSweep(-1) }
        btnGoRight.setOnClickListener { if (state == ScanState.CENTERED) startSweep(1) }
        btnGoUp.setOnClickListener    { if (state == ScanState.CENTERED) startClimb(1) }
        btnGoDown.setOnClickListener  { if (state == ScanState.CENTERED) startClimb(-1) }
        btnStop.setOnClickListener    { emergencyStop(); ToastUtils.showToast("STOPPED") }

        // P-Gain: 0.03 – 0.30
        seekPGain.max = 27
        seekPGain.progress = ((pGain - 0.03f) * 100).toInt()
        seekPGain.setOnSeekBarChangeListener(slider { p -> pGain = 0.03f + p / 100f; updateConfigText() })

        // D-Gain: 0.0 – 0.20
        seekDGain.max = 20
        seekDGain.progress = (dGain * 100).toInt()
        seekDGain.setOnSeekBarChangeListener(slider { p -> dGain = p / 100f; updateConfigText() })

        // Sweep cruise speed: 0.10–0.40 m/s in 0.05 m/s steps (max=6),
        // default 0.15 m/s. The pump reads sweepSpeedMps every tick so a
        // slider change is picked up on the next pump cycle (≤50 ms).
        seekSweepSpeed.max = 6
        seekSweepSpeed.progress = ((sweepSpeedMps - 0.10f) / 0.05f).toInt()
        seekSweepSpeed.setOnSeekBarChangeListener(slider { p ->
            sweepSpeedMps = 0.10f + p * 0.05f; updateConfigText()
        })

        // Climb distance per UP/DOWN press: 0.50–2.00 m in 0.25 m steps (max=6),
        // default 1.00 m. In POSITION mode this is the absolute altitude delta
        // the FC will fly to, NOT a speed — the FC chooses its own ramp.
        seekClimbSpeed.max = 6
        seekClimbSpeed.progress = ((climbDistanceM - 0.50f) / 0.25f).toInt()
        seekClimbSpeed.setOnSeekBarChangeListener(slider { p ->
            climbDistanceM = 0.50f + p * 0.25f; updateConfigText()
        })
    }

    private fun slider(onChange: (Int) -> Unit) = object : SeekBar.OnSeekBarChangeListener {
        override fun onProgressChanged(sb: SeekBar?, p: Int, f: Boolean) = onChange(p)
        override fun onStartTrackingTouch(sb: SeekBar?) {}
        override fun onStopTrackingTouch(sb: SeekBar?) {}
    }

    private fun setupCameraStream() {
        try {
            MediaDataCenter.getInstance().cameraStreamManager.addFrameListener(
                ComponentIndexType.LEFT_OR_MAIN, ICameraStreamManager.FrameFormat.NV21, frameListener)
        } catch (e: Exception) { mainHandler.post { tvState.text = "Camera failed — connect drone" } }
    }

    // ── Scan control ──

    private fun startScan() {
        currentTargetId = 0; currentLevel = 0; prevOffsetX = 0f
        safetyStopTriggered = false; sweepTargetAdvanced = false; centeringMissFrames = 0
        hoverStick()
        state = ScanState.CENTERING
        isTracking.set(true)
        ToastUtils.showToast("Scan started — center on marker 0")
        updateUI()
    }

    private fun startSweep(direction: Int) {
        movementStartTime  = System.currentTimeMillis()
        lastMarkerSeenTime = System.currentTimeMillis()
        safetyStopTriggered = false; centeringMissFrames = 0

        if (!sweepTargetAdvanced) {
            sweepTargetAdvanced = true
            currentTargetId = (currentTargetId + 1).coerceAtMost(totalMarkers - 1)
        }

        // VELOCITY-mode sweep: the pump now sends roll = sweepSpeedMps in
        // BODY-frame VELOCITY mode while state is SWEEPING_*. The FC handles
        // attitude and overrides its own position-hold to honor the velocity
        // command, which is what ANGLE mode couldn't do on a constant input.
        // No PD priming needed — that path only runs for centering.
        prevOffsetX   = 0f
        targetRollDeg = 0f
        inHoverLock   = true

        state = if (direction > 0) ScanState.SWEEPING_RIGHT else ScanState.SWEEPING_LEFT
        val dir = if (direction > 0) "RIGHT" else "LEFT"
        Log.i(TAG, "Sweep $dir at ${sweepSpeedMps} m/s, marker $currentTargetId")
        ToastUtils.showToast("Sweeping $dir → marker $currentTargetId (${sweepSpeedMps}m/s)")
        updateUI()
    }

    private fun startClimb(direction: Int) {
        climbStartTime     = System.currentTimeMillis()
        movementStartTime  = System.currentTimeMillis()
        lastMarkerSeenTime = System.currentTimeMillis()
        safetyStopTriggered = false; centeringMissFrames = 0

        // Capture the altitude at climb start so the POSITION-mode target is
        // stable for the duration of the climb (target = startAlt ± distance).
        // If we used currentAltitudeM directly the goal would chase the drone.
        climbStartAltitudeM = if (currentAltitudeM > 0.0) currentAltitudeM else
            try { FlightControllerKey.KeyAltitude.create().get(0.0) }
            catch (_: Throwable) { 0.0 }

        // VPS toggle is unsupported on Mini 4 Pro firmware (call fails with
        // "Not supported") — leaving the attempt in for forward compatibility,
        // but it's a no-op on this drone. POSITION mode does the heavy lifting.
        if (direction > 0) disableVpsForClimb()

        // Jump by one full level (markersPerLevel). With 2 markers per level
        // the right-side marker N steps to the right-side marker N+2 on the
        // next level up, not to the left-side marker N+1 on the SAME level
        // (which is already in frame and would short-circuit the climb).
        val markersPerLevel = if (numLevels > 0) (totalMarkers / numLevels).coerceAtLeast(1) else 1
        if (direction > 0) {
            currentLevel++
            currentTargetId = (currentTargetId + markersPerLevel).coerceAtMost(totalMarkers - 1)
        } else {
            currentLevel = (currentLevel - 1).coerceAtLeast(0)
            currentTargetId = (currentTargetId - markersPerLevel).coerceAtLeast(0)
        }
        sweepTargetAdvanced = true
        targetRollDeg = 0f
        state = if (direction > 0) ScanState.CLIMBING_UP else ScanState.CLIMBING_DOWN
        val dir = if (direction > 0) "UP" else "DOWN"
        val targetAlt = if (direction > 0) climbStartAltitudeM + climbDistanceM
                        else                (climbStartAltitudeM - climbDistanceM).coerceAtLeast(0.3)
        logBuffer.i(TAG, "Climb $dir start (POSITION): target marker=$currentTargetId (jump=$markersPerLevel), " +
            "from ${"%.2f".format(climbStartAltitudeM)}m to ${"%.2f".format(targetAlt)}m " +
            "(distance=${climbDistanceM}m)")
        ToastUtils.showToast("Climbing $dir → marker $currentTargetId (${climbSpeedMps}m/s)")
        updateUI()
    }

    // ── Takeoff auto-descent ──

    private fun beginTakeoffDescent() {
        ToastUtils.showToast("Descent: stabilize done, enabling VS")
        Log.i(TAG, "beginTakeoffDescent: isVSEnabled=$isVSEnabled")
        if (isVSEnabled) {
            startTakeoffDescentNow()
            return
        }
        VirtualStickManager.getInstance().enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                Log.i(TAG, "VS auto-enable: success")
                try { VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(true) }
                catch (t: Throwable) { Log.w(TAG, "adv mode (auto): ${t.message}") }
                isVSEnabled = true; inHoverLock = true
                mainHandler.post {
                    btnEnableVS.text = "VS ON"
                    tvSafety.visibility = View.GONE
                    ToastUtils.showToast("VS enabled (auto)")
                    startTakeoffDescentNow()
                }
            }
            override fun onFailure(e: IDJIError) {
                Log.w(TAG, "VS auto-enable failed: $e")
                mainHandler.post {
                    tvState.text = "Take off OK — VS enable failed; tap ENABLE VS"
                    ToastUtils.showToast("VS auto-enable failed: $e")
                }
            }
        })
    }

    private fun startTakeoffDescentNow() {
        val alt = try {
            FlightControllerKey.KeyAltitude.create().get(0.0)
        } catch (_: Throwable) { 0.0 }
        if (alt > 0.0) currentAltitudeM = alt
        Log.i(TAG, "startTakeoffDescentNow: sync alt=${"%.2f".format(alt)}m")

        if (alt > ALT_TOLERANCE_M && alt <= TARGET_TAKEOFF_HEIGHT_M + ALT_TOLERANCE_M) {
            ToastUtils.showToast("Descent skipped: already at %.2fm".format(alt))
            onTakeoffDescentComplete()
            return
        }
        targetRollDeg = 0f
        inHoverLock = true
        takeoffDescendStartMs = System.currentTimeMillis()
        state = ScanState.TAKEOFF_DESCENDING
        Log.i(TAG, "Descending to ${TARGET_TAKEOFF_HEIGHT_M}m at ${TAKEOFF_DESCEND_SPEED_MPS}m/s (from ${"%.2f".format(alt)}m)")
        ToastUtils.showToast("Descend: %.2fm → %.2fm @ %.2fm/s".format(
            alt, TARGET_TAKEOFF_HEIGHT_M, TAKEOFF_DESCEND_SPEED_MPS))
        updateUI()
    }

    // ── Height-limit cap management ──

    // Owner for the one-shot HeightLimit listener (separate from `this` so we
    // can cancel it independently of the fragment-wide cancelListen on destroy).
    private val heightLimitListenerOwner = Any()
    @Volatile private var heightLimitHandled: Boolean = false

    private fun raiseHeightLimitForScan() {
        if (originalHeightLimit != null) {
            ToastUtils.showToast("HeightLimit: already raised this session")
            return
        }
        heightLimitHandled = false

        // Fast path: many DJI keys are push-populated, but try a sync read
        // first in case the value is already cached.
        val sync = try {
            FlightControllerKey.KeyHeightLimit.create().get(-1)
        } catch (_: Throwable) { -1 }
        Log.i(TAG, "raiseHeightLimitForScan: sync read = $sync")
        if (sync >= 0) {
            heightLimitHandled = true
            ToastUtils.showToast("HeightLimit: read = ${sync}m")
            applyHeightLimitRaise(sync)
            return
        }

        // Slow path: wait for the first FC push of KeyHeightLimit (up to 5s).
        ToastUtils.showToast("HeightLimit: read pending, waiting...")
        Log.i(TAG, "HeightLimit sync read returned null — waiting for push")
        try {
            FlightControllerKey.KeyHeightLimit.create().listen(heightLimitListenerOwner) { v: Int? ->
                if (heightLimitHandled) return@listen
                val value = v ?: return@listen
                heightLimitHandled = true
                try { KeyManager.getInstance().cancelListen(heightLimitListenerOwner) }
                catch (_: Throwable) {}
                mainHandler.post { applyHeightLimitRaise(value) }
            }
        } catch (t: Throwable) {
            Log.w(TAG, "HeightLimit listener setup threw: ${t.message}")
            return
        }
        mainHandler.postDelayed({
            if (!heightLimitHandled) {
                heightLimitHandled = true
                try { KeyManager.getInstance().cancelListen(heightLimitListenerOwner) }
                catch (_: Throwable) {}
                Log.w(TAG, "HeightLimit listener timed out — no FC push in 5s")
                ToastUtils.showToast("HeightLimit read timeout — set it in Pilot")
            }
        }, 5000)
    }

    private fun applyHeightLimitRaise(current: Int) {
        telemetry.heightLimitM = current
        if (current >= TARGET_HEIGHT_LIMIT_M) {
            Log.i(TAG, "HeightLimit already ${current}m — no change")
            mainHandler.post { ToastUtils.showToast("HeightLimit already ${current}m — no change") }
            return
        }
        originalHeightLimit = current
        val key = KeyTools.createKey(FlightControllerKey.KeyHeightLimit)
        KeyManager.getInstance().setValue(key, TARGET_HEIGHT_LIMIT_M,
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    Log.i(TAG, "HeightLimit raised: ${current}m → ${TARGET_HEIGHT_LIMIT_M}m")
                    telemetry.heightLimitM = TARGET_HEIGHT_LIMIT_M
                    mainHandler.post {
                        ToastUtils.showToast("HeightLimit: ${current}m → ${TARGET_HEIGHT_LIMIT_M}m")
                    }
                }
                override fun onFailure(error: IDJIError) {
                    Log.w(TAG, "HeightLimit raise failed: $error")
                    originalHeightLimit = null
                    mainHandler.post { ToastUtils.showToast("HeightLimit raise failed: $error") }
                }
            })
    }

    // ── Flight control actions (callable from both phone buttons and dashboard) ──

    private fun doTakeoff() {
        basicAircraftControlVM.startTakeOff(object :
            CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                mainHandler.post {
                    tvState.text = "Hovering — auto-descend to %.2fm in %.0fs"
                        .format(TARGET_TAKEOFF_HEIGHT_M, TAKEOFF_STABILIZE_MS / 1000f)
                }
                ToastUtils.showToast("Take off OK")
                logBuffer.i(TAG, "Takeoff command accepted")
                raiseHeightLimitForScan()
                mainHandler.postDelayed({ beginTakeoffDescent() }, TAKEOFF_STABILIZE_MS)
            }
            override fun onFailure(e: IDJIError) {
                ToastUtils.showToast("Take off failed: $e")
                logBuffer.w(TAG, "Takeoff failed: $e")
            }
        })
    }

    private fun doEnableVS() {
        if (isVSEnabled) return
        VirtualStickManager.getInstance().enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                try { VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(true) }
                catch (t: Throwable) { Log.w(TAG, "adv mode: ${t.message}") }
                isVSEnabled = true; inHoverLock = true
                mainHandler.post {
                    btnEnableVS.text = "VS ON"
                    tvSafety.visibility = View.GONE
                    if (state == ScanState.IDLE) tvState.text = "VS ON — Set levels & Start Scan"
                    updateActionButtons()
                }
                ToastUtils.showToast("VS enabled")
                logBuffer.i(TAG, "VS enabled")
            }
            override fun onFailure(e: IDJIError) {
                ToastUtils.showToast("VS failed: $e")
                logBuffer.w(TAG, "VS enable failed: $e")
            }
        })
    }

    private fun doDisableVS() {
        // Cleanly hand control back to the RC without landing.
        if (!isVSEnabled) return
        emergencyStop()
        try { VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(false) }
        catch (_: Throwable) {}
        VirtualStickManager.getInstance().disableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                isVSEnabled = false
                mainHandler.post { btnEnableVS.text = "ENABLE VS" }
                logBuffer.i(TAG, "VS disabled")
            }
            override fun onFailure(e: IDJIError) {
                logBuffer.w(TAG, "VS disable failed: $e")
            }
        })
    }

    private fun doLand() {
        // Stop any in-progress mission, park the drone, drop VS, then land.
        if (telemetry.missionRunning) missionExecutor.stop()
        emergencyStop()
        try { VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(false) }
        catch (_: Throwable) {}
        VirtualStickManager.getInstance().disableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                isVSEnabled = false
                mainHandler.post { btnEnableVS.text = "ENABLE VS" }
                logBuffer.i(TAG, "Land: VS off, starting landing")
                basicAircraftControlVM.startLanding(object :
                    CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                    override fun onSuccess(t: EmptyMsg?) { mainHandler.post { tvState.text = "Landing..." } }
                    override fun onFailure(e: IDJIError) { mainHandler.post { tvState.text = "Land failed: $e" } }
                })
            }
            override fun onFailure(e: IDJIError) {
                // Even if VS disable failed, try to land anyway (best-effort).
                logBuffer.w(TAG, "Land: VS disable failed ($e), landing anyway")
                basicAircraftControlVM.startLanding(object :
                    CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                    override fun onSuccess(t: EmptyMsg?) {}
                    override fun onFailure(e2: IDJIError) {}
                })
            }
        })
    }

    // ── Mission scripting ──

    /** Bridge between MissionExecutor (background thread) and the fragment's
     *  state machine. All methods are called from the mission thread; field
     *  writes are safe because the underlying fields are @Volatile. */
    private val missionHost = object : MissionExecutor.Host {
        override fun currentAltitudeM(): Double = this@ArucoFollowFragment.currentAltitudeM

        override fun missionStartLeft()  { missionDriveSweep(-1) }
        override fun missionStartRight() { missionDriveSweep(+1) }

        override fun missionStartUp(distanceM: Float)   { missionDriveClimb(distanceM, up = true) }
        override fun missionStartDown(distanceM: Float) { missionDriveClimb(distanceM, up = false) }

        override fun missionStopAndHover() {
            state = ScanState.IDLE
            hoverStick()
        }

        override fun missionLand() {
            // doLand mutates UI / SDK state and must run on the main thread.
            mainHandler.post { doLand() }
        }

        override fun motorsOn(): Boolean = telemetry.motorsOn
    }

    /** Mirrors startSweep() but without marker advancement or capture-zone
     *  expectations — the executor terminates the step by time. */
    private fun missionDriveSweep(direction: Int) {
        movementStartTime  = System.currentTimeMillis()
        lastMarkerSeenTime = System.currentTimeMillis()   // silence safety
        safetyStopTriggered = false
        prevOffsetX   = 0f
        targetRollDeg = 0f
        inHoverLock   = true
        state = if (direction > 0) ScanState.SWEEPING_RIGHT else ScanState.SWEEPING_LEFT
    }

    /** Mirrors startClimb() in POSITION-mode terms but without marker
     *  advancement; distance is supplied by the mission step. */
    private fun missionDriveClimb(distanceM: Float, up: Boolean) {
        climbStartTime     = System.currentTimeMillis()
        movementStartTime  = System.currentTimeMillis()
        lastMarkerSeenTime = System.currentTimeMillis()
        safetyStopTriggered = false
        climbStartAltitudeM = if (currentAltitudeM > 0.0) currentAltitudeM else
            try { FlightControllerKey.KeyAltitude.create().get(0.0) }
            catch (_: Throwable) { 0.0 }
        climbDistanceM = distanceM
        targetRollDeg = 0f
        state = if (up) ScanState.CLIMBING_UP else ScanState.CLIMBING_DOWN
    }

    /** Dashboard WebSocket commands. Runs on the NanoWSD reader thread, so any
     *  call into MSDK callbacks that touch views must marshal via mainHandler. */
    private val dashboardCommandHandler = object : RackScanDashboardServer.CommandHandler {
        override fun onCommand(cmd: String, payload: org.json.JSONObject) {
            logBuffer.i(TAG, "Dashboard cmd: $cmd")
            when (cmd) {
                "takeoff"   -> mainHandler.post { doTakeoff() }
                "land"      -> mainHandler.post { doLand() }
                "enableVS"  -> mainHandler.post { doEnableVS() }
                "disableVS" -> mainHandler.post { doDisableVS() }
                "runMission" -> {
                    val arr = payload.optJSONArray("steps") ?: return
                    val steps = MissionStep.listFromJsonArray(arr)
                    val loop  = payload.optBoolean("loop", false)
                    if (steps.isEmpty()) {
                        logBuffer.w(TAG, "runMission rejected: empty/invalid steps")
                        return
                    }
                    if (!isVSEnabled) {
                        logBuffer.w(TAG, "runMission rejected: VS not enabled")
                        return
                    }
                    if (!missionExecutor.start(steps, loop)) {
                        logBuffer.w(TAG, "runMission rejected: a mission is already running")
                    }
                }
                "stopMission" -> missionExecutor.stop()
                else -> logBuffer.w(TAG, "Unknown dashboard cmd: $cmd")
            }
        }
    }

    // ── VPS (Vision Positioning System) toggle for climb ──

    private fun disableVpsForClimb() {
        if (telemetry.vpsDisabledByUs) return    // already off
        try {
            PerceptionManager.getInstance().setVisionPositioningEnabled(false,
                object : CommonCallbacks.CompletionCallback {
                    override fun onSuccess() {
                        telemetry.vpsDisabledByUs = true
                        logBuffer.i(TAG, "VPS disabled for climb")
                    }
                    override fun onFailure(error: IDJIError) {
                        logBuffer.w(TAG, "VPS disable failed: $error")
                    }
                })
        } catch (t: Throwable) {
            logBuffer.w(TAG, "VPS disable threw: ${t.message}")
        }
    }

    private fun restoreVpsIfDisabled() {
        if (!telemetry.vpsDisabledByUs) return
        try {
            PerceptionManager.getInstance().setVisionPositioningEnabled(true,
                object : CommonCallbacks.CompletionCallback {
                    override fun onSuccess() {
                        telemetry.vpsDisabledByUs = false
                        logBuffer.i(TAG, "VPS restored")
                    }
                    override fun onFailure(error: IDJIError) {
                        logBuffer.w(TAG, "VPS restore failed: $error")
                    }
                })
        } catch (t: Throwable) {
            logBuffer.w(TAG, "VPS restore threw: ${t.message}")
        }
    }

    private fun restoreHeightLimit() {
        val orig = originalHeightLimit ?: return
        originalHeightLimit = null
        try {
            KeyManager.getInstance().setValue(
                KeyTools.createKey(FlightControllerKey.KeyHeightLimit),
                orig,
                object : CommonCallbacks.CompletionCallback {
                    override fun onSuccess() { Log.i(TAG, "HeightLimit restored to ${orig}m") }
                    override fun onFailure(error: IDJIError) {
                        Log.w(TAG, "HeightLimit restore failed: $error")
                    }
                })
        } catch (t: Throwable) {
            Log.w(TAG, "HeightLimit restore threw: ${t.message}")
        }
    }

    private fun onTakeoffDescentComplete() {
        if (state != ScanState.TAKEOFF_DESCENDING) return
        state = ScanState.IDLE
        ToastUtils.showToast("At %.2fm — ready to scan".format(currentAltitudeM))
        Log.i(TAG, "Descent complete at ${"%.2f".format(currentAltitudeM)}m")
        updateUI()
    }

    private fun onTakeoffDescentTimeout() {
        if (state != ScanState.TAKEOFF_DESCENDING) return
        state = ScanState.IDLE
        Log.w(TAG, "Descent timeout at ${"%.2f".format(currentAltitudeM)}m (target ${TARGET_TAKEOFF_HEIGHT_M}m)")
        ToastUtils.showToast("Descent timeout @ %.2fm".format(currentAltitudeM))
        updateUI()
    }

    private fun onCentered() {
        state = ScanState.CENTERED; hoverStick(); prevOffsetX = 0f
        sweepTargetAdvanced = false; centeringMissFrames = 0
        if (currentTargetId >= totalMarkers - 1) {
            state = ScanState.COMPLETE; ToastUtils.showToast("Scan complete!")
        } else {
            ToastUtils.showToast("Centered on marker $currentTargetId — LEFT / RIGHT / UP")
        }
        updateUI()
    }

    private fun safetyStop(reason: String) {
        safetyStopTriggered = true; hoverStick(); state = ScanState.CENTERED
        // Reset sweepTargetAdvanced so the next LEFT/RIGHT still targets the
        // same marker (no further increment) but the state machine is clean.
        sweepTargetAdvanced = false
        restoreVpsIfDisabled()
        logBuffer.w(TAG, "SAFETY stop: $reason (alt=${"%.2f".format(currentAltitudeM)}m, target=$currentTargetId)")
        tvSafety.text = "SAFETY: $reason"; tvSafety.visibility = View.VISIBLE
        ToastUtils.showToast("Safety stop: $reason"); updateUI()
    }

    private fun emergencyStop() {
        isTracking.set(false); state = ScanState.IDLE; hoverStick(); prevOffsetX = 0f
        sweepTargetAdvanced = false; centeringMissFrames = 0
        restoreVpsIfDisabled()
        updateUI()
    }

    // ── Frame processing ──

    private fun processFrame(nv21: ByteArray, w: Int, h: Int) {
        if (!isTracking.get()) return
        isProcessing.set(true)
        try {
            val yuv = Mat(h + h / 2, w, CvType.CV_8UC1).also { it.put(0, 0, nv21) }
            val bgr = Mat()
            Imgproc.cvtColor(yuv, bgr, Imgproc.COLOR_YUV2BGR_NV21); yuv.release()

            val corners = mutableListOf<Mat>()
            val ids = Mat()
            arucoDetector.detectMarkers(bgr, corners, ids)

            val visIds = mutableListOf<Int>()
            if (ids.rows() > 0) for (i in 0 until ids.rows()) visIds.add(ids[i, 0][0].toInt())

            var detected = false; var offsetX = 0f; var markerSize = 0f
            for (i in 0 until ids.rows()) {
                if (ids[i, 0][0].toInt() == currentTargetId) {
                    detected = true
                    val mc = corners[i]
                    val cx = (0 until 4).map { mc[0, it][0] }.average().toFloat()
                    val dx = mc[0, 2][0] - mc[0, 0][0]; val dy = mc[0, 2][1] - mc[0, 0][1]
                    markerSize = Math.sqrt((dx * dx + dy * dy).toDouble()).toFloat()
                    offsetX = (cx - w / 2f) / (w / 2f)
                    break
                }
            }
            if (detected) lastMarkerSeenTime = System.currentTimeMillis()

            // Dashboard publish — frame-level fields
            telemetry.detected      = detected
            telemetry.offsetX       = offsetX
            telemetry.markerSize    = markerSize
            telemetry.visibleIdsCsv = visIds.joinToString(",")

            when (state) {
                ScanState.CENTERING, ScanState.FINE_TUNING -> {
                    if (detected) {
                        centeringMissFrames = 0
                        targetRollDeg = computeRollAngle(offsetX)
                        if (abs(offsetX) < CENTER_THRESHOLD) mainHandler.post { onCentered() }
                    } else {
                        if (++centeringMissFrames >= CENTERING_MISS_TOL) hoverStick()
                    }
                }
                ScanState.SWEEPING_LEFT, ScanState.SWEEPING_RIGHT -> {
                    // Sweep is driven by the pump in VELOCITY mode from
                    // sweepSpeedMps — no PD writes from the frame thread.
                    // Just watch for the target marker entering the capture
                    // zone and hand off to CENTERING, seeding prevOffsetX so
                    // the first centering D-term doesn't spike from 0.
                    // (Skipped during mission scripting — those are time-based.)
                    if (!telemetry.missionRunning &&
                        detected && abs(offsetX) < SWEEP_CAPTURE_ZONE) {
                        prevOffsetX = offsetX
                        targetRollDeg = 0f      // pump will see ANGLE 0° during the brief gap
                        state = ScanState.CENTERING
                        mainHandler.post { updateUI() }
                    }
                }
                ScanState.CLIMBING_UP, ScanState.CLIMBING_DOWN -> {
                    // Hand off to FINE_TUNING once the target marker enters
                    // the capture zone, after MIN_CLIMB_DURATION_MS, but only
                    // when we're operating under operator control — mission
                    // scripts terminate climbs by altitude target / time.
                    val elapsed = System.currentTimeMillis() - climbStartTime
                    if (!telemetry.missionRunning &&
                        elapsed >= MIN_CLIMB_DURATION_MS &&
                        detected && abs(offsetX) < SWEEP_CAPTURE_ZONE) {
                        logBuffer.i(TAG, "Climb capture: target=$currentTargetId offset=${"%.2f".format(offsetX)} " +
                            "after ${elapsed}ms, alt=${"%.2f".format(currentAltitudeM)}m → FINE_TUNING")
                        prevOffsetX = offsetX
                        targetRollDeg = 0f
                        state = ScanState.FINE_TUNING
                        restoreVpsIfDisabled()
                        mainHandler.post { updateUI() }
                    }
                }
                else -> {}
            }

            // Build preview
            val rgb = Mat()
            Imgproc.cvtColor(bgr, rgb, Imgproc.COLOR_BGR2RGB)
            val bmp = Bitmap.createBitmap(rgb.cols(), rgb.rows(), Bitmap.Config.ARGB_8888)
            Utils.matToBitmap(rgb, bmp)

            val fOff = offsetX; val fDet = detected; val fSz = markerSize
            val fIds = visIds.toList()
            val fRoll = targetRollDeg
            val fTicks = pumpTicks.get(); val now = System.currentTimeMillis()
            val fObs = nearestObstacleM
            val isSweeping = state == ScanState.SWEEPING_LEFT || state == ScanState.SWEEPING_RIGHT
            val isClimbing = state == ScanState.CLIMBING_UP || state == ScanState.CLIMBING_DOWN
            val fAlt = currentAltitudeM
            val inKick = now < kickEndsAtMs

            mainHandler.post {
                imgPreview.setImageBitmap(bmp)

                tvMarkerInfo.text = buildString {
                    append("T:$currentTargetId")
                    if (fDet) append(" FOUND off=%.2f sz=%.0f".format(fOff, fSz))
                    else append(" not found")
                    if (fIds.isNotEmpty()) append(" vis=$fIds")
                    append(" L${currentLevel + 1}")
                }

                val modeTag = when {
                    isSweeping                         -> "SWEEP %.2fm/s".format(sweepSpeedMps)
                    state == ScanState.CLIMBING_UP     -> "UP→%.2fm".format(climbStartAltitudeM + climbDistanceM)
                    state == ScanState.CLIMBING_DOWN   -> "DOWN→%.2fm".format((climbStartAltitudeM - climbDistanceM).coerceAtLeast(0.3))
                    state == ScanState.TAKEOFF_DESCENDING -> "DESCEND %.2fm/s".format(TAKEOFF_DESCEND_SPEED_MPS)
                    inKick                             -> "ANG* ${fRoll.toInt()}°"
                    else                               -> "ANG ${fRoll.toInt()}°"
                }
                tvStickDebug.text = "$modeTag alt=%.2fm pump=#$fTicks".format(fAlt)

                if (isSweeping || isClimbing) {
                    val age = (now - lastMarkerSeenTime) / 1000f
                    val obsStr = if (fObs in 0.01f..OBSTACLE_WARN_M) " ⚠OBS %.1fm".format(fObs) else ""
                    if (age > 1f || obsStr.isNotEmpty()) {
                        tvSafety.text = "No marker %.1fs/%.0fs%s".format(age, SAFETY_TIMEOUT_MS / 1000f, obsStr)
                        tvSafety.visibility = View.VISIBLE
                    } else if (!safetyStopTriggered) {
                        tvSafety.visibility = View.GONE
                    }
                } else if (!safetyStopTriggered) {
                    tvSafety.visibility = View.GONE
                }

                updateStateText()
            }

            bgr.release(); rgb.release(); ids.release(); corners.forEach { it.release() }
        } catch (e: Exception) {
            Log.e(TAG, "frame: ${e.message}", e)
        } finally {
            isProcessing.set(false)
        }
    }

    private fun computeRollAngle(offsetX: Float): Float {
        if (abs(offsetX) < DEAD_ZONE) { prevOffsetX = offsetX; inHoverLock = true; return 0f }
        val r = (pGain * offsetX + dGain * (offsetX - prevOffsetX))
            .coerceIn(-MAX_ROLL_RATIO, MAX_ROLL_RATIO)
        prevOffsetX = offsetX
        return r * MAX_ROLL_ANGLE_DEG
    }

    private fun updateUI() { mainHandler.post { updateActionButtons(); updateStateText() } }

    private fun updateActionButtons() {
        fun hideAllDirectional() {
            btnGoLeft.visibility = View.GONE; btnGoRight.visibility = View.GONE
            btnGoUp.visibility = View.GONE; btnGoDown.visibility = View.GONE
        }
        when (state) {
            ScanState.IDLE -> {
                btnStartScan.visibility = if (isVSEnabled) View.VISIBLE else View.GONE
                hideAllDirectional(); btnStop.visibility = View.GONE
                etLevels.isEnabled = true
            }
            ScanState.CENTERED -> {
                btnStartScan.visibility = View.GONE
                val ok = isVSEnabled
                btnGoLeft.visibility  = if (ok) View.VISIBLE else View.GONE
                btnGoRight.visibility = if (ok) View.VISIBLE else View.GONE
                btnGoUp.visibility    = if (ok) View.VISIBLE else View.GONE
                btnGoDown.visibility  = if (ok) View.VISIBLE else View.GONE
                btnStop.visibility = View.VISIBLE; etLevels.isEnabled = false
            }
            ScanState.SWEEPING_LEFT, ScanState.SWEEPING_RIGHT,
            ScanState.CLIMBING_UP, ScanState.CLIMBING_DOWN,
            ScanState.CENTERING, ScanState.FINE_TUNING,
            ScanState.TAKEOFF_DESCENDING -> {
                btnStartScan.visibility = View.GONE
                hideAllDirectional(); btnStop.visibility = View.VISIBLE
                etLevels.isEnabled = false
            }
            ScanState.COMPLETE -> {
                btnStartScan.visibility = View.VISIBLE
                hideAllDirectional(); btnStop.visibility = View.GONE
                etLevels.isEnabled = true
            }
        }
    }

    private fun updateStateText() {
        tvState.text = when (state) {
            ScanState.IDLE        -> "IDLE — Set levels & Start Scan"
            ScanState.CENTERING   -> "CENTERING on marker $currentTargetId..."
            ScanState.CENTERED    -> "CENTERED marker $currentTargetId (L${currentLevel + 1})\nChoose: LEFT / RIGHT / UP / DOWN"
            ScanState.SWEEPING_LEFT  -> "SWEEP LEFT → marker $currentTargetId (%.2fm/s)".format(sweepSpeedMps)
            ScanState.SWEEPING_RIGHT -> "SWEEP RIGHT → marker $currentTargetId (%.2fm/s)".format(sweepSpeedMps)
            ScanState.CLIMBING_UP   -> "CLIMB UP → marker $currentTargetId (target %.2fm)".format(climbStartAltitudeM + climbDistanceM)
            ScanState.CLIMBING_DOWN -> "CLIMB DOWN → marker $currentTargetId (target %.2fm)".format((climbStartAltitudeM - climbDistanceM).coerceAtLeast(0.3))
            ScanState.FINE_TUNING -> "FINE-TUNING marker $currentTargetId"
            ScanState.COMPLETE    -> "COMPLETE — $numLevels levels done"
            ScanState.TAKEOFF_DESCENDING -> "DESCEND to %.2fm (now %.2fm)".format(
                TARGET_TAKEOFF_HEIGHT_M, currentAltitudeM)
        }
    }

    private fun updateConfigText() {
        tvConfigInfo.text = "P:%.2f D:%.2f | Swp:%.2fm/s | Clb:%.2fm"
            .format(pGain, dGain, sweepSpeedMps, climbDistanceM)
    }

    // ── OA mode toggle ──

    private fun requestOaType(type: ObstacleAvoidanceType) {
        // Optimistic UI: tint the requested button immediately. The perception
        // info listener will confirm (or correct) the highlight on next update.
        val previous = currentOaType
        currentOaType = type
        refreshOaButtons()
        try {
            PerceptionManager.getInstance().setObstacleAvoidanceType(type, object :
                CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    Log.i(TAG, "OA mode → $type")
                    ToastUtils.showToast("OA: $type")
                }
                override fun onFailure(e: IDJIError) {
                    Log.w(TAG, "OA set $type failed: $e")
                    ToastUtils.showToast("OA set failed: $e")
                    currentOaType = previous
                    mainHandler.post { refreshOaButtons() }
                }
            })
        } catch (t: Throwable) {
            Log.w(TAG, "OA set $type threw: ${t.message}")
            currentOaType = previous
            refreshOaButtons()
        }
    }

    private fun refreshOaButtons() {
        // Active = green, inactive = gray. UNKNOWN/null → all gray (no claim).
        val active   = 0xFF2E7D32.toInt()
        val inactive = 0xFF555555.toInt()
        btnOaBrake .backgroundTintList = android.content.res.ColorStateList.valueOf(
            if (currentOaType == ObstacleAvoidanceType.BRAKE ) active else inactive)
        btnOaBypass.backgroundTintList = android.content.res.ColorStateList.valueOf(
            if (currentOaType == ObstacleAvoidanceType.BYPASS) active else inactive)
        btnOaClose .backgroundTintList = android.content.res.ColorStateList.valueOf(
            if (currentOaType == ObstacleAvoidanceType.CLOSE ) active else inactive)
    }

    override fun onDestroyView() {
        super.onDestroyView()
        isTracking.set(false); stopCommandLoop(); hoverStick()
        try { VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(false) }
        catch (_: Throwable) {}
        try { PerceptionManager.getInstance().removeObstacleDataListener(obstacleListener) }
        catch (_: Throwable) {}
        try { PerceptionManager.getInstance().removePerceptionInformationListener(perceptionInfoListener) }
        catch (_: Throwable) {}
        try { missionExecutor.shutdown() }
        catch (_: Throwable) {}
        try { dashboardServer?.stop() }
        catch (_: Throwable) {}
        dashboardServer = null
        restoreVpsIfDisabled()
        restoreHeightLimit()
        try { KeyManager.getInstance().cancelListen(this) }
        catch (_: Throwable) {}
        try { KeyManager.getInstance().cancelListen(heightLimitListenerOwner) }
        catch (_: Throwable) {}
        try { MediaDataCenter.getInstance().cameraStreamManager.removeFrameListener(frameListener) }
        catch (_: Exception) {}
        processingThread.quitSafely()
    }
}
