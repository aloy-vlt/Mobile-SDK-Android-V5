package dji.sampleV5.aircraft.models

import android.app.Application
import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.util.Log
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.MutableLiveData
import dji.sampleV5.aircraft.dashboard.DashboardServer
import dji.sampleV5.aircraft.rackmission.RackFlightController
import dji.sampleV5.aircraft.rackscan.MissionStep
import dji.sampleV5.aircraft.rackscan.MissionExecutor
import dji.sampleV5.aircraft.rackscan.RackScanLogBuffer
import dji.sampleV5.aircraft.rackscan.RackScanTelemetry
import dji.sampleV5.aircraft.rtsp.RtspStreamingService
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.et.create
import dji.v5.et.listen
import dji.v5.manager.KeyManager
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.net.Inet4Address
import java.net.NetworkInterface
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicLong

/**
 * Owns the Rack Scan Missioning dashboard. It reuses, rather than reinvents,
 * the building blocks proven by the ArUco Follow module:
 *   - Video : the same NV21 -> YuvImage -> JPEG -> binary-WebSocket path as
 *     [DashboardServerVM] (low latency, no transcode).
 *   - Server: the shared [DashboardServer], pointed at `assets/rackmission/`.
 *   - Mission engine: the SAME [MissionExecutor] + [MissionStep] +
 *     [RackScanTelemetry] + [RackScanLogBuffer] used by ArUco Follow — so a
 *     mission here is the identical "LEFT n s / RIGHT n s / UP n m / DOWN n m /
 *     HOVER n s / LAND" script, executed by the identical runner.
 *   - Flight : [RackFlightController] drives VirtualStick exactly as the ArUco
 *     pump does (sweeps in VELOCITY, climbs in POSITION).
 *   - RTSP   : delegated to the app-scoped [RtspStreamingService] singleton.
 *
 * The server pushes one combined JSON text frame per tick:
 *   { ts, telemetry:{...mission fields...}, rtsp:{...}, logs:[...] }
 * and interleaves binary JPEG frames for the raw feed.
 */
class RackScanMissionVM(app: Application) : AndroidViewModel(app), DashboardServer.CommandHandler {

    // ── State observed by the mobile fragment ──────────────────────────
    val serverState = MutableLiveData<ServerState>(ServerState.Stopped)
    val clientCount = MutableLiveData(0)
    val fps = MutableLiveData(0)
    val rtspRunning = MutableLiveData(false)
    val rtspUrl = MutableLiveData<String?>(null)

    private val logs = RackScanLogBuffer()
    private val telemetry = RackScanTelemetry()
    private val flight = RackFlightController(telemetry, logs)
    private val missionExecutor = MissionExecutor(flight, telemetry, logs)

    private var server: DashboardServer? = null
    private val encoderExec = Executors.newSingleThreadExecutor { r ->
        Thread(r, "rackmission-encoder").apply { isDaemon = true }
    }
    private val pushExec = Executors.newSingleThreadScheduledExecutor { r ->
        Thread(r, "rackmission-push").apply { isDaemon = true }
    }
    private val commandExec = Executors.newSingleThreadExecutor { r ->
        Thread(r, "rackmission-command").apply { isDaemon = true }
    }

    private val isEncoding = AtomicBoolean(false)
    private val framesThisSecond = AtomicInteger(0)
    private val lastFpsTick = AtomicLong(0)

    // Frame dimensions for the dashboard overlay are carried on telemetry via
    // these locals (RackScanTelemetry has no frame-size fields of its own).
    @Volatile private var frameW = 0
    @Volatile private var frameH = 0
    // RackScanTelemetry has no battery field, so we keep it here and splice it
    // into the pushed JSON alongside the telemetry snapshot.
    @Volatile private var battery = 0

    private val rtspListener = object : RtspStreamingService.Listener {
        override fun onStateChanged(state: RtspStreamingService.State) {
            when (state) {
                is RtspStreamingService.State.Running -> {
                    rtspRunning.postValue(true); rtspUrl.postValue(state.url)
                    logs.i(TAG, "RTSP streaming on ${state.url}")
                }
                is RtspStreamingService.State.Stopped -> {
                    rtspRunning.postValue(false); rtspUrl.postValue(null)
                    logs.i(TAG, "RTSP streaming stopped")
                }
                is RtspStreamingService.State.Error -> {
                    rtspRunning.postValue(false); rtspUrl.postValue(null)
                    logs.e(TAG, "RTSP error: ${state.message}")
                }
            }
        }
        override fun onClientCountChanged(count: Int) { /* surfaced via getClientCount */ }
    }

    private val frameListener = ICameraStreamManager.CameraFrameListener {
            frameData, offset, length, width, height, _ ->
        val srv = server ?: return@CameraFrameListener
        val aligning = flight.isAligning()
        // Encode for the feed only when someone's watching, but always process
        // frames while an ALIGN step is running (the centering loop needs them
        // even if no dashboard is open).
        if (srv.clientCount() == 0 && !aligning) return@CameraFrameListener
        if (!isEncoding.compareAndSet(false, true)) return@CameraFrameListener

        val copy = ByteArray(length)
        System.arraycopy(frameData, offset, copy, 0, length)

        encoderExec.execute {
            try {
                frameW = width; frameH = height
                if (flight.isAligning()) flight.processAlignFrame(copy, width, height)
                if (srv.clientCount() > 0) {
                    srv.pushJpegFrame(nv21ToJpeg(copy, width, height, JPEG_QUALITY))
                    tickFps()
                }
            } catch (t: Throwable) {
                Log.w(TAG, "frame handle failed: ${t.message}")
            } finally {
                isEncoding.set(false)
            }
        }
    }

    // ── Lifecycle ──────────────────────────────────────────────────────

    fun start(port: Int = DEFAULT_PORT) {
        if (server != null) return
        try {
            val srv = DashboardServer(port, getApplication<Application>().applicationContext, ASSET_ROOT)
            srv.commandHandler = this
            srv.start(NANO_SOCKET_TIMEOUT_MS, false)
            server = srv

            MediaDataCenter.getInstance().cameraStreamManager.addFrameListener(
                ComponentIndexType.LEFT_OR_MAIN,
                ICameraStreamManager.FrameFormat.NV21,
                frameListener,
            )
            registerTelemetryListeners()
            flight.start()
            RtspStreamingService.addListener(rtspListener)
            schedulePush()

            val url = "http://${getLanIp() ?: "<device-ip>"}:$port/"
            logs.i(TAG, "Dashboard server up — open $url")
            serverState.postValue(ServerState.Running(port, url))
        } catch (t: Throwable) {
            serverState.postValue(ServerState.Error(t.message ?: t.javaClass.simpleName))
            stop()
        }
    }

    fun stop() {
        missionExecutor.stop()
        flight.shutdown()
        MediaDataCenter.getInstance().cameraStreamManager.removeFrameListener(frameListener)
        KeyManager.getInstance().cancelListen(this)
        RtspStreamingService.removeListener(rtspListener)
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
        missionExecutor.shutdown()
        encoderExec.shutdownNow()
        pushExec.shutdownNow()
        commandExec.shutdownNow()
    }

    // ── Commands from the dashboard ────────────────────────────────────

    override fun onCommand(cmd: String, payload: JSONObject) {
        commandExec.execute {
            try {
                when (cmd) {
                    "takeoff"     -> flight.takeoff()
                    "land"        -> flight.land()
                    "enableVS"    -> flight.enableVS()
                    "disableVS"   -> flight.disableVS()
                    "runMission"  -> runMission(payload)
                    "stopMission" -> missionExecutor.stop()
                    "setSweepSpeed" -> flight.setSweepSpeed(payload.optDouble("mps", 0.3).toFloat())
                    "setArucoSize"  -> flight.setArucoSize(payload.optDouble("m", 0.1).toFloat())
                    "setStandoff"   -> flight.setStandoff(payload.optDouble("m", 1.0).toFloat())
                    "setCameraHfov" -> flight.setCameraHfov(payload.optDouble("deg", 82.0).toFloat())
                    "startRtsp"   -> startRtsp(payload.optInt("port", RtspStreamingService.DEFAULT_PORT))
                    "stopRtsp"    -> stopRtsp()
                    else -> Log.w(TAG, "unknown cmd: $cmd")
                }
            } catch (t: Throwable) {
                Log.e(TAG, "cmd $cmd failed: ${t.message}", t)
                logs.e(TAG, "Command '$cmd' failed: ${t.message}")
            }
        }
    }

    private fun runMission(payload: JSONObject) {
        val arr = payload.optJSONArray("steps")
        if (arr == null || arr.length() == 0) { logs.w(TAG, "runMission rejected: empty/invalid steps"); return }
        val steps = MissionStep.listFromJsonArray(arr)
        if (steps.isEmpty()) { logs.w(TAG, "runMission rejected: no valid steps"); return }
        if (!flight.isVSEnabled()) { logs.w(TAG, "runMission rejected: enable Virtual Stick first"); return }
        val loop = payload.optBoolean("loop", false)
        if (!missionExecutor.start(steps, loop)) logs.w(TAG, "runMission rejected: a mission is already running")
    }

    // ── RTSP ────────────────────────────────────────────────────────────

    private fun startRtsp(port: Int) {
        Thread({ RtspStreamingService.start(port) }, "rackmission-rtsp-start").start()
    }

    private fun stopRtsp() {
        Thread({ RtspStreamingService.stop() }, "rackmission-rtsp-stop").start()
    }

    // ── Telemetry ───────────────────────────────────────────────────────

    private fun registerTelemetryListeners() {
        FlightControllerKey.KeyAltitude.create().listen(this) { telemetry.altitudeM = it ?: 0.0 }
        FlightControllerKey.KeyAircraftVelocity.create().listen(this) { v ->
            v?.let { telemetry.velocityX = it.x; telemetry.velocityY = it.y; telemetry.velocityZ = it.z }
        }
        FlightControllerKey.KeyAreMotorsOn.create().listen(this) { telemetry.motorsOn = it == true }
        BatteryKey.KeyChargeRemainingInPercent.create().listen(this) { battery = it ?: 0 }
    }

    private fun schedulePush() {
        pushExec.scheduleAtFixedRate({
            val srv = server ?: return@scheduleAtFixedRate
            if (srv.clientCount() == 0) return@scheduleAtFixedRate
            telemetry.serverPushCount++
            telemetry.lastUpdateMs = System.currentTimeMillis()
            srv.pushTelemetry(composeJson())
        }, PUSH_INTERVAL_MS, PUSH_INTERVAL_MS, TimeUnit.MILLISECONDS)
    }

    private fun composeJson(): String = buildString(1400) {
        append('{')
        append("\"ts\":").append(System.currentTimeMillis()).append(',')
        append("\"telemetry\":").append(telemetry.snapshotJson()).append(',')
        append("\"frame\":{\"w\":").append(frameW).append(",\"h\":").append(frameH).append("},")
        append("\"battery\":").append(battery).append(',')
        append("\"rtsp\":{")
        append("\"running\":").append(RtspStreamingService.isRunning()).append(',')
        append("\"url\":").append(RtspStreamingService.getStreamUrl()?.let { "\"$it\"" } ?: "null").append(',')
        append("\"clients\":").append(RtspStreamingService.getClientCount())
        append("},")
        append("\"logs\":").append(logs.snapshotJson())
        append('}')
    }

    private fun tickFps() {
        val now = System.currentTimeMillis()
        val last = lastFpsTick.get()
        framesThisSecond.incrementAndGet()
        if (now - last >= 1000L && lastFpsTick.compareAndSet(last, now)) {
            fps.postValue(framesThisSecond.getAndSet(0))
            clientCount.postValue(server?.clientCount() ?: 0)
        }
    }

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
                if (addr is Inet4Address && !addr.isLoopbackAddress) return addr.hostAddress
            }
        }
        return null
    }

    sealed class ServerState {
        object Stopped : ServerState()
        data class Running(val port: Int, val url: String) : ServerState()
        data class Error(val message: String) : ServerState()
    }

    companion object {
        private const val TAG = "RackScanMissionVM"
        const val DEFAULT_PORT = 8083
        private const val ASSET_ROOT = "rackmission"
        private const val JPEG_QUALITY = 60
        private const val PUSH_INTERVAL_MS = 200L
        private const val NANO_SOCKET_TIMEOUT_MS = 0
    }
}
