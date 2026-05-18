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
import dji.sampleV5.aircraft.models.VirtualStickVM
import dji.sampleV5.aircraft.util.ToastUtils
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
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

        private const val CLIMB_SPEED_MPS      = 0.3f
        private const val SWEEP_CAPTURE_ZONE   = 0.55f   // normalised offset
        private const val CENTERING_MISS_TOL   = 5       // frames before hoverStick
        private const val OBSTACLE_WARN_M      = 1.5f
    }

    enum class ScanState {
        IDLE, CENTERING, CENTERED,
        SWEEPING_LEFT, SWEEPING_RIGHT,
        CLIMBING, FINE_TUNING, COMPLETE
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
    private lateinit var seekClimbHeight: SeekBar
    private lateinit var btnTakeOff: Button
    private lateinit var btnEnableVS: Button
    private lateinit var btnLand: Button
    private lateinit var btnStartScan: Button
    private lateinit var btnGoLeft: Button
    private lateinit var btnGoRight: Button
    private lateinit var btnGoUp: Button
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

    // Climb
    private var climbHeight    = 0.5f
    private var climbStartTime = 0L
    private var climbDurationMs = 0L

    // Sweep leg tracking
    private var sweepTargetAdvanced  = false
    private var centeringMissFrames  = 0

    // Safety
    @Volatile private var lastMarkerSeenTime = 0L
    private var movementStartTime    = 0L
    private var safetyStopTriggered  = false

    // ── ANGLE MODE TARGETS (written by frame thread, read by pump thread) ──
    // Single roll target covers all states: PD centering values AND sweep angles.
    @Volatile private var targetRollDeg: Float   = 0f
    @Volatile private var targetThrottleMps: Float = 0f

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
    // Centering / Fine-tuning → ANGLE mode (BODY frame).
    //   Each frame, computeRollAngle(realOffsetX) writes targetRollDeg. The
    //   pump sends that with the shared kickstart for sub-KICK_RATIO outputs.
    //   This is the path proven by the Dashboard's aruco loop — short
    //   corrections in a closed loop where offsetX naturally varies.
    //
    // Sweeping → VELOCITY mode (BODY frame).
    //   ANGLE mode with a constant attitude command fights the FC's position-
    //   hold outer loop (drone moves a bit, FC counter-tilts, drone stalls).
    //   VELOCITY mode tells the FC to *travel* at sweepSpeedMps and the FC
    //   handles attitude internally, including overriding position-hold.
    //   This is the API DJI tuned for sustained directional motion.
    //
    // Climbing → ANGLE roll = 0, verticalThrottle = CLIMB_SPEED_MPS.
    // ═══════════════════════════════════════════════════════════════════

    private fun startCommandLoop() {
        commandTimer = Timer("StickPump", true)
        commandTimer?.scheduleAtFixedRate(object : TimerTask() {
            override fun run() {
                if (!isVSEnabled) return
                if (state == ScanState.IDLE || state == ScanState.COMPLETE) return

                val now = System.currentTimeMillis()
                val isSweeping = state == ScanState.SWEEPING_LEFT ||
                                 state == ScanState.SWEEPING_RIGHT

                try {
                    val param = VirtualStickFlightControlParam().apply {
                        rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                        yawControlMode            = YawControlMode.ANGULAR_VELOCITY
                        verticalControlMode       = VerticalControlMode.VELOCITY
                        yaw                       = 0.0
                        verticalThrottle          = targetThrottleMps.toDouble()
                        if (isSweeping) {
                            // DJI MSDK V5 convention: in VELOCITY mode, `pitch`
                            // and `roll` are linear velocities ALONG the named
                            // axis. Roll axis is longitudinal (fwd/back), pitch
                            // axis is lateral (left/right). So lateral motion
                            // is driven via `pitch`, not `roll` — opposite of
                            // ANGLE-mode where `roll` is body-roll attitude.
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
                } catch (t: Throwable) {
                    Log.w(TAG, "pump: ${t.message}")
                }

                // Safety watchdogs
                if (state == ScanState.SWEEPING_LEFT || state == ScanState.SWEEPING_RIGHT) {
                    if (now - lastMarkerSeenTime > SAFETY_TIMEOUT_MS &&
                        now - movementStartTime > SAFETY_TIMEOUT_MS) {
                        mainHandler.post { safetyStop("No marker ${SAFETY_TIMEOUT_MS / 1000}s") }
                    }
                }
                if (state == ScanState.CLIMBING &&
                    now - climbStartTime > climbDurationMs + SAFETY_TIMEOUT_MS) {
                    mainHandler.post { safetyStop("Climb timeout") }
                }
            }
        }, 0, COMMAND_INTERVAL_MS)
    }

    private fun stopCommandLoop() { commandTimer?.cancel(); commandTimer = null }

    private fun hoverStick() {
        targetRollDeg     = 0f
        targetThrottleMps = 0f
        inHoverLock       = true
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
        seekClimbHeight = view.findViewById(R.id.seek_climb_height)
        btnTakeOff   = view.findViewById(R.id.btn_takeoff)
        btnEnableVS  = view.findViewById(R.id.btn_enable_vs)
        btnLand      = view.findViewById(R.id.btn_land)
        btnStartScan = view.findViewById(R.id.btn_start_scan)
        btnGoLeft    = view.findViewById(R.id.btn_go_left)
        btnGoRight   = view.findViewById(R.id.btn_go_right)
        btnGoUp      = view.findViewById(R.id.btn_go_up)
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

        btnTakeOff.setOnClickListener {
            basicAircraftControlVM.startTakeOff(object :
                CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) {
                    mainHandler.post { tvState.text = "Hovering — Enable VS next" }
                    ToastUtils.showToast("Take off OK")
                }
                override fun onFailure(e: IDJIError) { ToastUtils.showToast("Take off failed: $e") }
            })
        }

        btnEnableVS.setOnClickListener {
            if (isVSEnabled) return@setOnClickListener
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
                }
                override fun onFailure(e: IDJIError) { ToastUtils.showToast("VS failed: $e") }
            })
        }

        btnLand.setOnClickListener {
            emergencyStop()
            try { VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(false) }
            catch (_: Throwable) {}
            VirtualStickManager.getInstance().disableVirtualStick(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    isVSEnabled = false
                    mainHandler.post { btnEnableVS.text = "ENABLE VS" }
                    basicAircraftControlVM.startLanding(object :
                        CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                        override fun onSuccess(t: EmptyMsg?) { mainHandler.post { tvState.text = "Landing..." } }
                        override fun onFailure(e: IDJIError) { mainHandler.post { tvState.text = "Land failed: $e" } }
                    })
                }
                override fun onFailure(e: IDJIError) {
                    basicAircraftControlVM.startLanding(object :
                        CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                        override fun onSuccess(t: EmptyMsg?) {}
                        override fun onFailure(e2: IDJIError) {}
                    })
                }
            })
        }

        btnStartScan.setOnClickListener {
            val lv = etLevels.text.toString().toIntOrNull()
            if (lv == null || lv < 1 || lv > 12) { ToastUtils.showToast("Enter levels 1-12"); return@setOnClickListener }
            numLevels = lv; totalMarkers = numLevels * 2; startScan()
        }

        btnGoLeft.setOnClickListener  { if (state == ScanState.CENTERED) startSweep(-1) }
        btnGoRight.setOnClickListener { if (state == ScanState.CENTERED) startSweep(1) }
        btnGoUp.setOnClickListener    { if (state == ScanState.CENTERED) startClimb() }
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

        // Climb height: 0.2 – 1.2 m
        seekClimbHeight.max = 10
        seekClimbHeight.progress = ((climbHeight - 0.2f) * 10).toInt()
        seekClimbHeight.setOnSeekBarChangeListener(slider { p -> climbHeight = 0.2f + p / 10f; updateConfigText() })
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
        prevOffsetX       = 0f
        targetRollDeg     = 0f
        targetThrottleMps = 0f
        inHoverLock       = true

        state = if (direction > 0) ScanState.SWEEPING_RIGHT else ScanState.SWEEPING_LEFT
        val dir = if (direction > 0) "RIGHT" else "LEFT"
        Log.i(TAG, "Sweep $dir at ${sweepSpeedMps} m/s, marker $currentTargetId")
        ToastUtils.showToast("Sweeping $dir → marker $currentTargetId (${sweepSpeedMps}m/s)")
        updateUI()
    }

    private fun startClimb() {
        climbDurationMs    = ((climbHeight / CLIMB_SPEED_MPS) * 1000).toLong()
        climbStartTime     = System.currentTimeMillis()
        movementStartTime  = System.currentTimeMillis()
        lastMarkerSeenTime = System.currentTimeMillis()
        safetyStopTriggered = false; centeringMissFrames = 0
        currentLevel++
        currentTargetId = (currentTargetId + 1).coerceAtMost(totalMarkers - 1)
        sweepTargetAdvanced = true
        targetRollDeg     = 0f
        targetThrottleMps = CLIMB_SPEED_MPS
        state = ScanState.CLIMBING
        Log.i(TAG, "Climb ${CLIMB_SPEED_MPS}m/s ${climbDurationMs}ms → marker $currentTargetId")
        ToastUtils.showToast("Climbing to level ${currentLevel + 1}")
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
        Log.w(TAG, "SAFETY: $reason")
        tvSafety.text = "SAFETY: $reason"; tvSafety.visibility = View.VISIBLE
        ToastUtils.showToast("Safety stop: $reason"); updateUI()
    }

    private fun emergencyStop() {
        isTracking.set(false); state = ScanState.IDLE; hoverStick(); prevOffsetX = 0f
        sweepTargetAdvanced = false; centeringMissFrames = 0; updateUI()
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

            when (state) {
                ScanState.CENTERING, ScanState.FINE_TUNING -> {
                    if (detected) {
                        centeringMissFrames = 0
                        targetRollDeg = computeRollAngle(offsetX); targetThrottleMps = 0f
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
                    if (detected && abs(offsetX) < SWEEP_CAPTURE_ZONE) {
                        prevOffsetX = offsetX
                        targetRollDeg = 0f      // pump will see ANGLE 0° during the brief gap
                        state = ScanState.CENTERING
                        mainHandler.post { updateUI() }
                    }
                }
                ScanState.CLIMBING -> {
                    val elapsed = System.currentTimeMillis() - climbStartTime
                    if (elapsed >= climbDurationMs || (detected && elapsed > 500)) {
                        state = ScanState.FINE_TUNING; hoverStick(); mainHandler.post { updateUI() }
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
            val fRoll = targetRollDeg; val fThr = targetThrottleMps
            val fTicks = pumpTicks.get(); val now = System.currentTimeMillis()
            val fObs = nearestObstacleM
            val isSweeping = state == ScanState.SWEEPING_LEFT || state == ScanState.SWEEPING_RIGHT
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
                    isSweeping -> "SWEEP %.2fm/s".format(sweepSpeedMps)
                    inKick     -> "ANG* ${fRoll.toInt()}°"
                    else       -> "ANG ${fRoll.toInt()}°"
                }
                tvStickDebug.text = "$modeTag thr=${fThr} pump=#$fTicks"

                if (isSweeping || state == ScanState.CLIMBING) {
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
        when (state) {
            ScanState.IDLE -> {
                btnStartScan.visibility = if (isVSEnabled) View.VISIBLE else View.GONE
                btnGoLeft.visibility = View.GONE; btnGoRight.visibility = View.GONE
                btnGoUp.visibility = View.GONE; btnStop.visibility = View.GONE
                etLevels.isEnabled = true
            }
            ScanState.CENTERED -> {
                btnStartScan.visibility = View.GONE
                val ok = isVSEnabled
                btnGoLeft.visibility  = if (ok) View.VISIBLE else View.GONE
                btnGoRight.visibility = if (ok) View.VISIBLE else View.GONE
                btnGoUp.visibility    = if (ok) View.VISIBLE else View.GONE
                btnStop.visibility = View.VISIBLE; etLevels.isEnabled = false
            }
            ScanState.SWEEPING_LEFT, ScanState.SWEEPING_RIGHT,
            ScanState.CLIMBING, ScanState.CENTERING, ScanState.FINE_TUNING -> {
                btnStartScan.visibility = View.GONE
                btnGoLeft.visibility = View.GONE; btnGoRight.visibility = View.GONE
                btnGoUp.visibility = View.GONE; btnStop.visibility = View.VISIBLE
                etLevels.isEnabled = false
            }
            ScanState.COMPLETE -> {
                btnStartScan.visibility = View.VISIBLE
                btnGoLeft.visibility = View.GONE; btnGoRight.visibility = View.GONE
                btnGoUp.visibility = View.GONE; btnStop.visibility = View.GONE
                etLevels.isEnabled = true
            }
        }
    }

    private fun updateStateText() {
        tvState.text = when (state) {
            ScanState.IDLE        -> "IDLE — Set levels & Start Scan"
            ScanState.CENTERING   -> "CENTERING on marker $currentTargetId..."
            ScanState.CENTERED    -> "CENTERED marker $currentTargetId (L${currentLevel + 1})\nChoose: LEFT / RIGHT / UP"
            ScanState.SWEEPING_LEFT  -> "SWEEP LEFT → marker $currentTargetId (%.2fm/s)".format(sweepSpeedMps)
            ScanState.SWEEPING_RIGHT -> "SWEEP RIGHT → marker $currentTargetId (%.2fm/s)".format(sweepSpeedMps)
            ScanState.CLIMBING    -> "CLIMBING L${currentLevel + 1} %.1fs/%.1fs".format(
                (System.currentTimeMillis() - climbStartTime) / 1000f, climbDurationMs / 1000f)
            ScanState.FINE_TUNING -> "FINE-TUNING marker $currentTargetId"
            ScanState.COMPLETE    -> "COMPLETE — $numLevels levels done"
        }
    }

    private fun updateConfigText() {
        tvConfigInfo.text = "P:%.2f D:%.2f | Swp:%.2fm/s | Clb:${CLIMB_SPEED_MPS}m/s H:${climbHeight}m"
            .format(pGain, dGain, sweepSpeedMps)
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
        try { MediaDataCenter.getInstance().cameraStreamManager.removeFrameListener(frameListener) }
        catch (_: Exception) {}
        processingThread.quitSafely()
    }
}
