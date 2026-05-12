package dji.sampleV5.aircraft.models

import android.app.Application
import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.util.Log
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.MutableLiveData
import dji.sampleV5.aircraft.dashboard.DashboardServer
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.et.action
import dji.v5.et.create
import dji.v5.et.listen
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.perception.PerceptionManager
import dji.v5.manager.aircraft.perception.data.ObstacleAvoidanceType
import dji.v5.manager.aircraft.perception.data.ObstacleData
import dji.v5.manager.aircraft.perception.data.PerceptionDirection
import dji.v5.manager.aircraft.perception.data.PerceptionInfo
import dji.v5.manager.aircraft.perception.listener.ObstacleDataListener
import dji.v5.manager.aircraft.perception.listener.PerceptionInformationListener
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import org.json.JSONObject
import org.opencv.android.OpenCVLoader
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.objdetect.ArucoDetector
import org.opencv.objdetect.DetectorParameters
import org.opencv.objdetect.Objdetect
import java.io.ByteArrayOutputStream
import java.net.Inet4Address
import java.net.NetworkInterface
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.abs

/**
 * Owns the embedded [DashboardServer] lifecycle, the MSDK subscriptions that
 * feed it (camera frames + flight telemetry), and an optional ArUco tracking
 * pipeline that can run on top of the same camera stream.
 *
 * Two modes:
 *   - Stream mode (default): NV21 -> YuvImage -> JPEG -> broadcast.
 *   - ArUco mode: NV21 -> BGR Mat -> detect markers -> (PD controller drives
 *     virtual stick if tracking is enabled) -> draw overlay -> JPEG.
 *
 * Commands from the browser arrive as JSON text frames and are dispatched
 * through [DashboardServer.CommandHandler].
 */
class DashboardServerVM(app: Application) : AndroidViewModel(app), DashboardServer.CommandHandler {

    val serverState = MutableLiveData<ServerState>(ServerState.Stopped)
    val clientCount = MutableLiveData(0)
    val fps = MutableLiveData(0)

    private var server: DashboardServer? = null
    private val encoderExec = Executors.newSingleThreadExecutor { r ->
        Thread(r, "dashboard-encoder").apply { isDaemon = true }
    }
    private val telemetryExec = Executors.newSingleThreadScheduledExecutor { r ->
        Thread(r, "dashboard-telemetry").apply { isDaemon = true }
    }
    private val commandExec = Executors.newSingleThreadExecutor { r ->
        Thread(r, "dashboard-command").apply { isDaemon = true }
    }
    private val stickPumpExec = Executors.newSingleThreadScheduledExecutor { r ->
        Thread(r, "dashboard-stick-pump").apply { isDaemon = true }
    }

    private val isEncoding = AtomicBoolean(false)
    private val framesThisSecond = AtomicInteger(0)
    private val lastFpsTick = AtomicLong(0)

    // Tracking state — written from command thread, read from encoder thread.
    @Volatile private var arucoMode: Boolean = false
    @Volatile private var arucoReady: Boolean = false
    private val isTracking = AtomicBoolean(false)
    @Volatile private var pGain: Float = DEFAULT_P_GAIN
    @Volatile private var dGain: Float = DEFAULT_D_GAIN
    @Volatile private var autoLand: Boolean = false
    @Volatile private var vsEnabled: Boolean = false
    @Volatile private var prevOffsetX: Float = 0f
    @Volatile private var centerStartTime: Long = 0L
    @Volatile private var isCentered: Boolean = false

    // Target roll angle (degrees) written by the encoder thread, flushed to
    // MSDK by the 20 Hz pump via sendVirtualStickAdvancedParam in ANGLE mode.
    // Attitude commands bypass both the basic-stick noise filter AND the
    // velocity-tracker filter that make smaller correction inputs invisible
    // to the FC after a hover settles. Tradeoff: obstacle-avoidance is not
    // applied to direct attitude commands — caller is responsible for safety.
    @Volatile private var targetRollDeg: Float = 0f

    // Kickstart state: whenever PD output drops to exactly zero (marker in
    // deadzone), the flight controller enters tight position-hold. Subsequent
    // small PD outputs get filtered as noise. We track that "we just came out
    // of zero" and apply a boosted magnitude for KICK_DURATION_MS to break the
    // FC out of hover-hold, then return to normal PD behaviour.
    @Volatile private var inHoverLock: Boolean = true
    @Volatile private var kickEndsAtMs: Long = 0L

    // Latest snapshot from PerceptionManager listeners. Both PerceptionInfo
    // (settings + which directions are working) and ObstacleData (live sensor
    // distances) get rolled into the telemetry stream so the dashboard can
    // show whether OA is active and detecting things.
    @Volatile private var perceptionInfo: PerceptionInfo? = null
    @Volatile private var obstacleData: ObstacleData? = null

    private val perceptionInfoListener = PerceptionInformationListener { info ->
        perceptionInfo = info
    }
    private val obstacleDataListener = ObstacleDataListener { data ->
        obstacleData = data
    }

    private var arucoDetector: ArucoDetector? = null

    private val telemetry = TelemetrySnapshot()

    private val frameListener = ICameraStreamManager.CameraFrameListener {
            frameData, offset, length, width, height, _ ->
        val srv = server ?: return@CameraFrameListener
        if (srv.clientCount() == 0) return@CameraFrameListener
        if (!isEncoding.compareAndSet(false, true)) return@CameraFrameListener

        val copy = ByteArray(length)
        System.arraycopy(frameData, offset, copy, 0, length)

        encoderExec.execute {
            try {
                telemetry.frameWidth = width
                telemetry.frameHeight = height
                val jpeg = if (arucoMode && arucoReady) {
                    processArucoFrame(copy, width, height)
                } else {
                    // Stream-only mode: no detection, no marker bbox.
                    telemetry.tracking.markerCornersJson = "null"
                    nv21ToJpeg(copy, width, height, JPEG_QUALITY)
                }
                srv.pushJpegFrame(jpeg)
                tickFps()
            } catch (t: Throwable) {
                Log.w(TAG, "frame encode failed: ${t.message}")
            } finally {
                isEncoding.set(false)
            }
        }
    }

    fun start(port: Int = DEFAULT_PORT) {
        if (server != null) return
        try {
            val srv = DashboardServer(port, getApplication<Application>().applicationContext)
            srv.commandHandler = this
            srv.start(NanoWSD_SOCKET_TIMEOUT_MS, false)
            server = srv

            MediaDataCenter.getInstance().cameraStreamManager.addFrameListener(
                ComponentIndexType.LEFT_OR_MAIN,
                ICameraStreamManager.FrameFormat.NV21,
                frameListener,
            )
            registerTelemetryListeners()
            registerPerceptionListeners()
            scheduleTelemetryBroadcast()
            scheduleStickPump()

            val url = "http://${getLanIp() ?: "<device-ip>"}:$port/"
            serverState.postValue(ServerState.Running(port, url))
        } catch (t: Throwable) {
            serverState.postValue(ServerState.Error(t.message ?: t.javaClass.simpleName))
            stop()
        }
    }

    fun stop() {
        // Make sure we cut virtual stick output before tearing anything down.
        zeroSticks()
        isTracking.set(false)
        arucoMode = false
        autoLand = false
        vsEnabled = false

        MediaDataCenter.getInstance().cameraStreamManager.removeFrameListener(frameListener)
        try {
            PerceptionManager.getInstance().removePerceptionInformationListener(perceptionInfoListener)
            PerceptionManager.getInstance().removeObstacleDataListener(obstacleDataListener)
        } catch (_: Throwable) { /* fine if never registered */ }
        perceptionInfo = null
        obstacleData = null
        KeyManager.getInstance().cancelListen(this)
        server?.commandHandler = null
        server?.stop()
        server = null
        serverState.postValue(ServerState.Stopped)
        clientCount.postValue(0)
        fps.postValue(0)
    }

    override fun onCleared() {
        super.onCleared()
        stop()
        encoderExec.shutdownNow()
        telemetryExec.shutdownNow()
        commandExec.shutdownNow()
        stickPumpExec.shutdownNow()
    }

    private fun scheduleStickPump() {
        stickPumpExec.scheduleAtFixedRate({
            // Only push when we're actively in control. When VS is off or tracking is
            // off we don't want to be writing to MSDK at all — the RC has authority.
            if (!vsEnabled || !isTracking.get()) return@scheduleAtFixedRate
            try {
                val param = VirtualStickFlightControlParam().apply {
                    // ANGLE mode: roll value is an absolute attitude command
                    // (degrees of tilt) that bypasses the FC's stick and
                    // velocity-tracker filters. This is what actually unsticks
                    // small corrections after a hover settle.
                    rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                    rollPitchControlMode = RollPitchControlMode.ANGLE
                    yawControlMode = YawControlMode.ANGULAR_VELOCITY
                    verticalControlMode = VerticalControlMode.VELOCITY
                    pitch = 0.0
                    roll = targetRollDeg.toDouble()  // degrees, body-frame tilt
                    yaw = 0.0
                    verticalThrottle = 0.0
                }
                VirtualStickManager.getInstance().sendVirtualStickAdvancedParam(param)
            } catch (_: Throwable) { /* MSDK may transiently reject — ignore */ }
        }, 0L, STICK_PUMP_INTERVAL_MS, TimeUnit.MILLISECONDS)
    }

    // ------------------------------------------------------------------
    // Commands (DashboardServer.CommandHandler)
    // ------------------------------------------------------------------

    override fun onCommand(cmd: String, payload: JSONObject) {
        commandExec.execute {
            try {
                when (cmd) {
                    "takeoff"    -> doTakeoff()
                    "land"       -> doLand()
                    "enableVS"   -> doEnableVS()
                    "disableVS"  -> doDisableVS()
                    "startTrack" -> doStartTrack()
                    "stopTrack"  -> doStopTrack()
                    "setGain"    -> {
                        payload.optDouble("pGain", Double.NaN).takeUnless { it.isNaN() }
                            ?.let { pGain = it.toFloat().coerceIn(0f, 1f) }
                        payload.optDouble("dGain", Double.NaN).takeUnless { it.isNaN() }
                            ?.let { dGain = it.toFloat().coerceIn(0f, 1f) }
                    }
                    "setArucoMode" -> setArucoMode(payload.optBoolean("on", false))
                    "setAutoLand"  -> autoLand = payload.optBoolean("on", false)
                    "setOaType"    -> setOaType(payload.optString("type", "CLOSE"))
                    "setOaEnabled" -> setOaEnabled(
                        payload.optString("direction", "HORIZONTAL"),
                        payload.optBoolean("on", false),
                    )
                    else -> Log.w(TAG, "unknown cmd: $cmd")
                }
            } catch (t: Throwable) {
                Log.e(TAG, "cmd $cmd failed: ${t.message}", t)
            }
        }
    }

    private fun doTakeoff() {
        FlightControllerKey.KeyStartTakeoff.create().action({
            Log.i(TAG, "takeoff success")
        }, { e: IDJIError ->
            Log.w(TAG, "takeoff failed: $e")
        })
    }

    private fun doLand() {
        FlightControllerKey.KeyStartAutoLanding.create().action({
            Log.i(TAG, "land success")
        }, { e: IDJIError ->
            Log.w(TAG, "land failed: $e")
        })
    }

    private fun doEnableVS() {
        VirtualStickManager.getInstance().enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                vsEnabled = true
                // Switch to advanced mode so the pump can send explicit attitude
                // commands (roll angle in degrees) via sendVirtualStickAdvancedParam.
                // This bypasses the FC's stick-input noise filter.
                try {
                    VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(true)
                    Log.i(TAG, "VS enabled (advanced mode)")
                } catch (t: Throwable) {
                    Log.w(TAG, "advanced mode toggle failed: ${t.message}")
                }
            }
            override fun onFailure(error: IDJIError) {
                Log.w(TAG, "VS enable failed: $error")
            }
        })
    }

    private fun doDisableVS() {
        zeroSticks()
        // A fresh enable always starts in the locked state — first non-zero
        // output should kick.
        inHoverLock = true
        kickEndsAtMs = 0L
        try {
            VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(false)
        } catch (_: Throwable) { /* fine if already off */ }
        VirtualStickManager.getInstance().disableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                vsEnabled = false
                Log.i(TAG, "VS disabled")
            }
            override fun onFailure(error: IDJIError) {
                Log.w(TAG, "VS disable failed: $error")
            }
        })
    }

    private fun doStartTrack() {
        prevOffsetX = 0f
        centerStartTime = 0L
        isCentered = false
        inHoverLock = true  // first PD output will kick
        kickEndsAtMs = 0L
        isTracking.set(true)
    }

    private fun doStopTrack() {
        isTracking.set(false)
        zeroSticks()
        prevOffsetX = 0f
        centerStartTime = 0L
        isCentered = false
        inHoverLock = true
        kickEndsAtMs = 0L
    }

    private fun setOaType(type: String) {
        val parsed = try {
            ObstacleAvoidanceType.valueOf(type.uppercase())
        } catch (_: Throwable) {
            Log.w(TAG, "unknown oa type: $type")
            return
        }
        PerceptionManager.getInstance().setObstacleAvoidanceType(parsed, object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { Log.i(TAG, "OA type set: $parsed") }
            override fun onFailure(error: IDJIError) { Log.w(TAG, "OA type set failed: $error") }
        })
    }

    private fun setOaEnabled(direction: String, on: Boolean) {
        val dir = try {
            PerceptionDirection.valueOf(direction.uppercase())
        } catch (_: Throwable) {
            Log.w(TAG, "unknown direction: $direction")
            return
        }
        PerceptionManager.getInstance().setObstacleAvoidanceEnabled(on, dir, object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { Log.i(TAG, "OA $dir enabled=$on") }
            override fun onFailure(error: IDJIError) { Log.w(TAG, "OA $dir enable=$on failed: $error") }
        })
    }

    private fun registerPerceptionListeners() {
        try {
            PerceptionManager.getInstance().addPerceptionInformationListener(perceptionInfoListener)
            PerceptionManager.getInstance().addObstacleDataListener(obstacleDataListener)
        } catch (t: Throwable) {
            Log.w(TAG, "PerceptionManager listener register failed: ${t.message}")
        }
    }

    private fun setArucoMode(on: Boolean) {
        if (on && !arucoReady) {
            if (!OpenCVLoader.initLocal()) {
                Log.e(TAG, "OpenCV init failed")
                return
            }
            val dict = Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50)
            arucoDetector = ArucoDetector(dict, DetectorParameters())
            arucoReady = true
            Log.i(TAG, "ArUco detector ready")
        }
        arucoMode = on
        if (!on) {
            doStopTrack()
        }
    }

    private fun zeroSticks() {
        targetRollDeg = 0f
        try {
            // Belt-and-braces: also reset the basic stick positions in case
            // anything reads them while we're between modes.
            VirtualStickManager.getInstance().rightStick.horizontalPosition = 0
            VirtualStickManager.getInstance().rightStick.verticalPosition = 0
            VirtualStickManager.getInstance().leftStick.horizontalPosition = 0
            VirtualStickManager.getInstance().leftStick.verticalPosition = 0
        } catch (_: Throwable) { /* VS may not be enabled yet — fine */ }
    }

    // ------------------------------------------------------------------
    // ArUco frame processing
    // ------------------------------------------------------------------

    private fun processArucoFrame(nv21: ByteArray, width: Int, height: Int): ByteArray {
        // Detection only — no drawing on the frame. The browser renders all
        // overlays (crosshair, marker bbox, deadzone, etc.) from the marker
        // corners we publish in telemetry.
        //
        // ArUco only needs a single-channel grayscale image, and NV21's Y plane
        // is already exactly that — so we wrap the whole NV21 buffer in a Mat
        // and submat the top w*h bytes for detection. No cvtColor cost, no
        // BGR Mat, no Bitmap.
        val yuvMat = Mat(height + height / 2, width, CvType.CV_8UC1)
        yuvMat.put(0, 0, nv21)
        val grayMat = yuvMat.submat(0, height, 0, width)

        val corners = mutableListOf<Mat>()
        val ids = Mat()
        arucoDetector?.detectMarkers(grayMat, corners, ids)

        var detected = false
        var offsetX = 0f
        var markerSize = 0f
        var cornersJson = "null"

        if (ids.rows() > 0) {
            for (i in 0 until ids.rows()) {
                if (ids[i, 0][0].toInt() == TARGET_MARKER_ID) {
                    detected = true
                    val mc = corners[i]
                    // Read 4 (x,y) corner points as 8 floats.
                    val pts = FloatArray(8)
                    for (j in 0 until 4) {
                        val pt = mc[0, j]
                        pts[j * 2]     = pt[0].toFloat()
                        pts[j * 2 + 1] = pt[1].toFloat()
                    }
                    cornersJson = buildString(80) {
                        append('[')
                        for (j in 0 until 4) {
                            if (j > 0) append(',')
                            append('[').append(pts[j * 2]).append(',').append(pts[j * 2 + 1]).append(']')
                        }
                        append(']')
                    }
                    val cx = (pts[0] + pts[2] + pts[4] + pts[6]) / 4f
                    val dx = pts[4] - pts[0]
                    val dy = pts[5] - pts[1]
                    markerSize = kotlin.math.sqrt(dx * dx + dy * dy)
                    offsetX = (cx - width / 2f) / (width / 2f)
                    break
                }
            }
        }

        grayMat.release()
        ids.release()
        corners.forEach { it.release() }
        yuvMat.release()

        // PD controller — frame thread only updates the target. The dedicated
        // stickPumpExec flushes it to MSDK at a fixed 20 Hz so the command
        // stream stays fresh regardless of how slow this frame pipeline gets.
        var rollPercent = 0f
        if (isTracking.get()) {
            if (detected) {
                val rollDeg = computeRollAngle(offsetX)
                rollPercent = (rollDeg / MAX_ROLL_ANGLE_DEG) * 100f
                targetRollDeg = rollDeg
                checkCentered(offsetX)
            } else {
                targetRollDeg = 0f
                centerStartTime = 0L
                isCentered = false
                prevOffsetX = 0f
            }
        } else {
            targetRollDeg = 0f
            centerStartTime = 0L
            isCentered = false
        }

        telemetry.tracking.update(
            arucoMode = true,
            isTracking = isTracking.get(),
            detected = detected,
            offsetX = offsetX,
            markerSize = markerSize,
            isCentered = isCentered,
            centerHoldMs = if (isCentered) System.currentTimeMillis() - centerStartTime else 0L,
            pGain = pGain,
            dGain = dGain,
            rollPercent = rollPercent,
            rollDeg = targetRollDeg,
            kicking = System.currentTimeMillis() < kickEndsAtMs,
            autoLand = autoLand,
            vsEnabled = vsEnabled,
        )
        telemetry.tracking.markerCornersJson = cornersJson

        // Fast JPEG encode — same path as stream mode.
        return nv21ToJpeg(nv21, width, height, JPEG_QUALITY)
    }

    private fun computeRollAngle(offsetX: Float): Float {
        if (abs(offsetX) < DEAD_ZONE) {
            prevOffsetX = offsetX
            inHoverLock = true
            return 0f
        }
        val pTerm = pGain * offsetX
        val dTerm = dGain * (offsetX - prevOffsetX)
        prevOffsetX = offsetX
        var rollRatio = pTerm + dTerm

        // Kickstart kept as a safety belt — in ANGLE mode the FC honours
        // attitude commands directly, but a small initial boost still helps
        // make the first correction visible after a long hover.
        if (inHoverLock) {
            inHoverLock = false
            kickEndsAtMs = System.currentTimeMillis() + KICK_DURATION_MS
        }
        if (System.currentTimeMillis() < kickEndsAtMs &&
            abs(rollRatio) > 0f && abs(rollRatio) < KICK_RATIO) {
            rollRatio = if (rollRatio > 0) KICK_RATIO else -KICK_RATIO
        }

        rollRatio = rollRatio.coerceIn(-MAX_ROLL_RATIO, MAX_ROLL_RATIO)
        return rollRatio * MAX_ROLL_ANGLE_DEG
    }

    private fun checkCentered(offsetX: Float) {
        // Use DEAD_ZONE (not DEAD_ZONE * 2) so "centered" means the PD controller is
        // outputting zero. Otherwise the timer can fire while the drone is still
        // actively correcting, surprising the operator.
        if (abs(offsetX) < DEAD_ZONE) {
            if (!isCentered) {
                isCentered = true
                centerStartTime = System.currentTimeMillis()
            } else if (autoLand &&
                       System.currentTimeMillis() - centerStartTime >= CENTER_HOLD_TIME_MS) {
                Log.i(TAG, "Centered ${CENTER_HOLD_TIME_MS}ms — auto-land armed, landing")
                isTracking.set(false)
                zeroSticks()
                VirtualStickManager.getInstance().disableVirtualStick(object : CommonCallbacks.CompletionCallback {
                    override fun onSuccess() { vsEnabled = false; doLand() }
                    override fun onFailure(error: IDJIError) { doLand() }
                })
            }
        } else {
            isCentered = false
            centerStartTime = 0L
        }
    }

    // ------------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------------

    private fun registerTelemetryListeners() {
        FlightControllerKey.KeyAltitude.create().listen(this) {
            telemetry.altitude = it ?: 0.0
        }
        FlightControllerKey.KeyAircraftLocation.create().listen(this) {
            it?.let { loc ->
                telemetry.latitude = loc.latitude
                telemetry.longitude = loc.longitude
            }
        }
        FlightControllerKey.KeyAircraftVelocity.create().listen(this) { v ->
            v?.let {
                telemetry.velocityX = it.x
                telemetry.velocityY = it.y
                telemetry.velocityZ = it.z
            }
        }
        FlightControllerKey.KeyAreMotorsOn.create().listen(this) {
            telemetry.motorsOn = it == true
        }
        BatteryKey.KeyChargeRemainingInPercent.create().listen(this) {
            telemetry.batteryPercent = it ?: 0
        }
    }

    private fun scheduleTelemetryBroadcast() {
        telemetryExec.scheduleAtFixedRate({
            val srv = server ?: return@scheduleAtFixedRate
            if (srv.clientCount() == 0) return@scheduleAtFixedRate
            // Keep arucoMode/tracking flags fresh in case ArUco is off
            // (frame pipeline only writes them when in ArUco mode).
            if (!arucoMode) {
                telemetry.tracking.update(
                    arucoMode = false,
                    isTracking = isTracking.get(),
                    detected = false,
                    offsetX = 0f,
                    markerSize = 0f,
                    isCentered = false,
                    centerHoldMs = 0L,
                    pGain = pGain,
                    dGain = dGain,
                    rollPercent = 0f,
                    rollDeg = 0f,
                    kicking = false,
                    autoLand = autoLand,
                    vsEnabled = vsEnabled,
                )
            }
            srv.pushTelemetry(composeTelemetryJson())
        }, TELEMETRY_INTERVAL_MS, TELEMETRY_INTERVAL_MS, TimeUnit.MILLISECONDS)
    }

    private fun tickFps() {
        val now = System.currentTimeMillis()
        val last = lastFpsTick.get()
        framesThisSecond.incrementAndGet()
        if (now - last >= 1000L && lastFpsTick.compareAndSet(last, now)) {
            val f = framesThisSecond.getAndSet(0)
            fps.postValue(f)
            clientCount.postValue(server?.clientCount() ?: 0)
        }
    }

    // ------------------------------------------------------------------
    // Encoding helpers
    // ------------------------------------------------------------------

    private fun nv21ToJpeg(nv21: ByteArray, width: Int, height: Int, quality: Int): ByteArray {
        val img = YuvImage(nv21, ImageFormat.NV21, width, height, null)
        val baos = ByteArrayOutputStream(width * height / 4)
        img.compressToJpeg(Rect(0, 0, width, height), quality, baos)
        return baos.toByteArray()
    }

    private fun getLanIp(): String? {
        val ifs = NetworkInterface.getNetworkInterfaces() ?: return null
        for (nif in ifs) {
            if (!nif.isUp || nif.isLoopback || nif.isVirtual) continue
            for (addr in nif.inetAddresses) {
                if (addr is Inet4Address && !addr.isLoopbackAddress) {
                    return addr.hostAddress
                }
            }
        }
        return null
    }

    sealed class ServerState {
        object Stopped : ServerState()
        data class Running(val port: Int, val url: String) : ServerState()
        data class Error(val message: String) : ServerState()
    }

    private class TrackingSnapshot {
        @Volatile var arucoMode: Boolean = false
        @Volatile var isTracking: Boolean = false
        @Volatile var detected: Boolean = false
        @Volatile var offsetX: Float = 0f
        @Volatile var markerSize: Float = 0f
        @Volatile var isCentered: Boolean = false
        @Volatile var centerHoldMs: Long = 0L
        @Volatile var pGain: Float = DEFAULT_P_GAIN
        @Volatile var dGain: Float = DEFAULT_D_GAIN
        @Volatile var rollPercent: Float = 0f
        @Volatile var rollDeg: Float = 0f
        @Volatile var kicking: Boolean = false
        // Pre-serialised JSON fragment so we don't re-format the FloatArray on
        // every broadcast tick. Either "null" or "[[x,y],[x,y],[x,y],[x,y]]"
        // in frame pixel coordinates.
        @Volatile var markerCornersJson: String = "null"
        @Volatile var autoLand: Boolean = false
        @Volatile var vsEnabled: Boolean = false

        fun update(
            arucoMode: Boolean, isTracking: Boolean, detected: Boolean,
            offsetX: Float, markerSize: Float, isCentered: Boolean, centerHoldMs: Long,
            pGain: Float, dGain: Float, rollPercent: Float, rollDeg: Float, kicking: Boolean,
            autoLand: Boolean, vsEnabled: Boolean,
        ) {
            this.arucoMode = arucoMode
            this.isTracking = isTracking
            this.detected = detected
            this.offsetX = offsetX
            this.markerSize = markerSize
            this.isCentered = isCentered
            this.centerHoldMs = centerHoldMs
            this.pGain = pGain
            this.dGain = dGain
            this.rollPercent = rollPercent
            this.rollDeg = rollDeg
            this.kicking = kicking
            this.autoLand = autoLand
            this.vsEnabled = vsEnabled
        }
    }

    private class TelemetrySnapshot {
        @Volatile var altitude: Double = 0.0
        @Volatile var latitude: Double = 0.0
        @Volatile var longitude: Double = 0.0
        @Volatile var velocityX: Double = 0.0
        @Volatile var velocityY: Double = 0.0
        @Volatile var velocityZ: Double = 0.0
        @Volatile var batteryPercent: Int = 0
        @Volatile var motorsOn: Boolean = false
        // Frame dimensions of the latest streamed frame, needed by the browser
        // to set its SVG overlay viewBox so overlays line up with the video.
        @Volatile var frameWidth: Int = 0
        @Volatile var frameHeight: Int = 0
        val tracking = TrackingSnapshot()

        fun toJson(): String = buildString(320) {
            append('{')
            append("\"ts\":").append(System.currentTimeMillis()).append(',')
            append("\"altitude\":").append(altitude).append(',')
            append("\"latitude\":").append(latitude).append(',')
            append("\"longitude\":").append(longitude).append(',')
            append("\"velocity\":{")
            append("\"x\":").append(velocityX).append(',')
            append("\"y\":").append(velocityY).append(',')
            append("\"z\":").append(velocityZ)
            append("},")
            append("\"battery\":").append(batteryPercent).append(',')
            append("\"motorsOn\":").append(motorsOn).append(',')
            append("\"frameWidth\":").append(frameWidth).append(',')
            append("\"frameHeight\":").append(frameHeight).append(',')
            append("\"tracking\":{")
            append("\"arucoMode\":").append(tracking.arucoMode).append(',')
            append("\"isTracking\":").append(tracking.isTracking).append(',')
            append("\"detected\":").append(tracking.detected).append(',')
            append("\"offsetX\":").append(tracking.offsetX).append(',')
            append("\"markerSize\":").append(tracking.markerSize).append(',')
            append("\"isCentered\":").append(tracking.isCentered).append(',')
            append("\"centerHoldMs\":").append(tracking.centerHoldMs).append(',')
            append("\"pGain\":").append(tracking.pGain).append(',')
            append("\"dGain\":").append(tracking.dGain).append(',')
            append("\"rollPercent\":").append(tracking.rollPercent).append(',')
            append("\"rollDeg\":").append(tracking.rollDeg).append(',')
            append("\"kicking\":").append(tracking.kicking).append(',')
            append("\"autoLand\":").append(tracking.autoLand).append(',')
            append("\"vsEnabled\":").append(tracking.vsEnabled).append(',')
            // DEAD_ZONE constant — kept in sync with companion's DEAD_ZONE = 0.05f.
            // Browser uses this to draw the deadzone band on the video overlay.
            append("\"deadZone\":0.05,")
            append("\"markerCorners\":").append(tracking.markerCornersJson)
            append('}')
            append('}')
        }
    }

    /** Append the obstacle-avoidance / perception snapshot to an in-progress
     *  telemetry JSON object. Splices in just before the closing brace.
     *  Lives on the outer class so it can read perceptionInfo/obstacleData. */
    private fun appendSafetyJson(sb: StringBuilder) {
        val info = perceptionInfo
        val data = obstacleData
        sb.append("\"safety\":{")
        sb.append("\"oaType\":\"").append(info?.obstacleAvoidanceType?.name ?: "UNKNOWN").append("\",")
        sb.append("\"enabled\":{")
            .append("\"horizontal\":").append(info?.isHorizontalObstacleAvoidanceEnabled ?: false).append(',')
            .append("\"upward\":").append(info?.isUpwardObstacleAvoidanceEnabled ?: false).append(',')
            .append("\"downward\":").append(info?.isDownwardObstacleAvoidanceEnabled ?: false)
        sb.append("},")
        sb.append("\"working\":{")
            .append("\"forward\":").append(info?.forwardObstacleAvoidanceWorking ?: false).append(',')
            .append("\"backward\":").append(info?.backwardObstacleAvoidanceWorking ?: false).append(',')
            .append("\"left\":").append(info?.leftSideObstacleAvoidanceWorking ?: false).append(',')
            .append("\"right\":").append(info?.rightSideObstacleAvoidanceWorking ?: false).append(',')
            .append("\"down\":").append(info?.downwardObstacleAvoidanceWorking ?: false)
        sb.append("},")
        sb.append("\"distance\":{")
            .append("\"horizontal\":").append(minDistanceJson(data?.horizontalObstacleDistance)).append(',')
            .append("\"upward\":").append(minDistanceJson(data?.upwardObstacleDistance)).append(',')
            .append("\"downward\":").append(minDistanceJson(data?.downwardObstacleDistance))
        sb.append('}')
        sb.append('}')
    }

    /** Merge the base telemetry JSON (which ends with `}`) with the safety
     *  fragment by replacing the trailing brace with `,<safety>}`. Cheaper
     *  than re-parsing or duplicating the JSON-building code. */
    private fun composeTelemetryJson(): String {
        val base = telemetry.toJson()
        val merged = StringBuilder(base.length + 220)
        merged.append(base, 0, base.length - 1)  // drop trailing }
        merged.append(',')
        appendSafetyJson(merged)
        merged.append('}')
        return merged.toString()
    }

    /** ObstacleData distance fields can be a scalar, primitive array, or List
     *  depending on the airframe. Reduce to a single nearest-distance number
     *  for JSON; emit `null` when there's no usable reading. */
    private fun minDistanceJson(v: Any?): String = when (v) {
        null -> "null"
        is Number -> v.toString()
        is DoubleArray -> v.toList().filter { it > 0 }.minOrNull()?.toString() ?: "null"
        is FloatArray -> v.toList().filter { it > 0 }.minOrNull()?.toString() ?: "null"
        is IntArray -> v.toList().filter { it > 0 }.minOrNull()?.toString() ?: "null"
        is List<*> -> v.filterIsInstance<Number>().map { it.toDouble() }.filter { it > 0 }.minOrNull()?.toString() ?: "null"
        else -> "null"
    }

    companion object {
        private const val TAG = "DashboardServerVM"
        const val DEFAULT_PORT = 8080
        private const val JPEG_QUALITY = 60
        private const val TELEMETRY_INTERVAL_MS = 200L
        private const val NanoWSD_SOCKET_TIMEOUT_MS = 0

        const val TARGET_MARKER_ID = 0
        const val DEFAULT_P_GAIN = 0.15f
        const val DEFAULT_D_GAIN = 0.10f
        private const val DEAD_ZONE = 0.05f
        // Output cap: PD ratio is clamped to [-MAX_ROLL_RATIO, +MAX_ROLL_RATIO]
        // and then scaled to degrees via MAX_ROLL_ANGLE_DEG. With ratio cap 0.6
        // and angle cap 15°, the strongest correction is ±9° tilt — fast enough
        // to track but gentle enough to be safe.
        private const val MAX_ROLL_RATIO = 0.6f
        private const val MAX_ROLL_ANGLE_DEG = 15.0f
        // Kick floor (also as a ratio) — minimum non-zero output during the
        // kick window. ~3.75° tilt is unmistakable to the FC.
        private const val KICK_RATIO = 0.25f
        private const val KICK_DURATION_MS = 250L
        private const val CENTER_HOLD_TIME_MS = 3000L
        private const val STICK_PUMP_INTERVAL_MS = 50L  // 20 Hz
    }
}
