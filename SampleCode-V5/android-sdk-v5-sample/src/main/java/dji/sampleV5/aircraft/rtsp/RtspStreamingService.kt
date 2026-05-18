package dji.sampleV5.aircraft.rtsp

import android.util.Log
import java.net.Inet4Address
import java.net.NetworkInterface
import java.util.concurrent.CopyOnWriteArraySet
import java.util.concurrent.atomic.AtomicReference

/**
 * App-scoped singleton that owns the RTSP server lifecycle and the MSDK
 * camera subscription that feeds it. Decoupled from any specific UI so other
 * features (or future automation) can call [start] / [stop] / [getState]
 * without taking a dependency on the streaming fragment.
 *
 * Same camera index as the LAN Dashboard (LEFT_OR_MAIN) — MSDK fans out
 * receive-stream listeners independently per call site, so the dashboard's
 * NV21 frame listener and this RTSP server's NAL receive-stream listener can
 * coexist on the same camera.
 */
object RtspStreamingService {

    sealed class State {
        object Stopped : State()
        data class Running(val port: Int, val url: String) : State()
        data class Error(val message: String) : State()
    }

    interface Listener {
        fun onStateChanged(state: State)
        fun onClientCountChanged(count: Int)
    }

    private val state = AtomicReference<State>(State.Stopped)
    private val listeners = CopyOnWriteArraySet<Listener>()
    private var source: MainCameraSource? = null
    private var server: RtspServer? = null

    fun getState(): State = state.get()
    fun isRunning(): Boolean = state.get() is State.Running
    fun getClientCount(): Int = server?.clientCount() ?: 0
    fun getStreamUrl(): String? = (state.get() as? State.Running)?.url

    fun addListener(l: Listener) {
        listeners.add(l)
        // Push current state immediately so newly-attached UIs don't show
        // stale defaults.
        l.onStateChanged(state.get())
        l.onClientCountChanged(getClientCount())
    }

    fun removeListener(l: Listener) { listeners.remove(l) }

    @Synchronized
    fun start(port: Int = DEFAULT_PORT, streamPath: String = DEFAULT_STREAM_PATH): Boolean {
        if (state.get() is State.Running) return true
        return try {
            val src = MainCameraSource()
            if (!src.start()) {
                publish(State.Error("failed to subscribe to MSDK camera stream — connect the drone first"))
                return false
            }
            val srv = RtspServer(port, src, streamPath).apply {
                onClientCountChanged = { n ->
                    for (l in listeners) {
                        try { l.onClientCountChanged(n) } catch (_: Throwable) {}
                    }
                }
            }
            srv.start()
            source = src
            server = srv
            val url = "rtsp://${getLanIp() ?: "<device-ip>"}:$port$streamPath"
            publish(State.Running(port, url))
            Log.i(TAG, "RTSP streaming started: $url")
            true
        } catch (t: Throwable) {
            Log.e(TAG, "start failed: ${t.message}", t)
            publish(State.Error(t.message ?: t.javaClass.simpleName))
            stop()
            false
        }
    }

    @Synchronized
    fun stop() {
        try { server?.stop() } catch (_: Throwable) {}
        try { source?.stop() } catch (_: Throwable) {}
        server = null
        source = null
        publish(State.Stopped)
        for (l in listeners) {
            try { l.onClientCountChanged(0) } catch (_: Throwable) {}
        }
    }

    private fun publish(s: State) {
        state.set(s)
        for (l in listeners) {
            try { l.onStateChanged(s) } catch (_: Throwable) {}
        }
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

    const val DEFAULT_PORT = 8554
    const val DEFAULT_STREAM_PATH = "/fpv"
    private const val TAG = "RtspStreamingService"
}
