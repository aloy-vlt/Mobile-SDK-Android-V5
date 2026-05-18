package dji.sampleV5.aircraft.rackscan

import android.content.Context
import android.util.Log
import fi.iki.elonen.NanoHTTPD
import fi.iki.elonen.NanoWSD
import org.json.JSONObject
import java.io.IOException
import java.io.InputStream
import java.util.concurrent.CopyOnWriteArraySet
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit

/**
 * Embedded HTTP + WebSocket server for the rack-scan dashboard. Mirrors the
 * shape of [dji.sampleV5.aircraft.dashboard.DashboardServer] but is purpose-
 * built for the [dji.sampleV5.aircraft.pages.ArucoFollowFragment] state
 * inspection workflow:
 *
 * - GET /             -> assets/rackscan/index.html
 * - GET /<path>       -> assets/rackscan/<path>
 * - WS  /ws           -> server pushes a single JSON object every
 *                        [PUSH_INTERVAL_MS] containing {telemetry, logs}.
 *                        No commands accepted (read-only dashboard).
 *
 * Owned by ArucoFollowFragment; started in onViewCreated, stopped in
 * onDestroyView. Telemetry and log writes happen from the fragment's frame
 * thread and pump thread; the dashboard's push loop runs on its own
 * single-thread executor so neither side blocks the other.
 */
class RackScanDashboardServer(
    port: Int = DEFAULT_PORT,
    private val appContext: Context,
    private val telemetry: RackScanTelemetry,
    private val logs: RackScanLogBuffer,
) : NanoWSD(port) {

    /** Commands sent from dashboard clients. The fragment is the handler. */
    interface CommandHandler {
        fun onCommand(cmd: String, payload: JSONObject)
    }
    var commandHandler: CommandHandler? = null

    private val clients = CopyOnWriteArraySet<RackScanSocket>()
    private val pushExec = Executors.newSingleThreadScheduledExecutor { r ->
        Thread(r, "RackScanDashPush").apply { isDaemon = true }
    }
    @Volatile private var pushTask: ScheduledFuture<*>? = null

    fun clientCount(): Int = clients.size

    override fun start() {
        super.start(SOCKET_READ_TIMEOUT_MS, false)
        pushTask = pushExec.scheduleAtFixedRate({
            if (clients.isEmpty()) return@scheduleAtFixedRate
            telemetry.serverPushCount++
            val msg = buildString(2700) {
                append("{\"telemetry\":").append(telemetry.snapshotJson())
                append(",\"logs\":").append(logs.snapshotJson())
                append('}')
            }
            for (c in clients) {
                try { c.send(msg) } catch (_: Throwable) { clients.remove(c) }
            }
        }, 0L, PUSH_INTERVAL_MS, TimeUnit.MILLISECONDS)
        Log.i(TAG, "started on port=${listeningPort}")
    }

    override fun stop() {
        pushTask?.cancel(false); pushTask = null
        pushExec.shutdownNow()
        for (c in clients) try { c.close(NanoWSD.WebSocketFrame.CloseCode.GoingAway, "stop", false) } catch (_: Throwable) {}
        clients.clear()
        super.stop()
        Log.i(TAG, "stopped")
    }

    override fun openWebSocket(handshake: IHTTPSession): WebSocket = RackScanSocket(handshake)

    override fun serveHttp(session: IHTTPSession): Response {
        val rawPath = session.uri.trimStart('/')
        val path = if (rawPath.isEmpty()) "index.html" else rawPath
        if (path.contains("..")) {
            return newFixedLengthResponse(Response.Status.FORBIDDEN, MIME_PLAINTEXT, "forbidden")
        }
        val assetPath = "rackscan/$path"
        return try {
            val stream: InputStream = appContext.assets.open(assetPath)
            val mime = mimeOf(path)
            newChunkedResponse(Response.Status.OK, mime, stream).apply {
                addHeader("Cache-Control", "no-store, no-cache, must-revalidate")
                addHeader("Pragma", "no-cache")
                addHeader("Expires", "0")
            }
        } catch (e: IOException) {
            newFixedLengthResponse(Response.Status.NOT_FOUND, MIME_PLAINTEXT, "not found: $path")
        }
    }

    private fun mimeOf(path: String): String = when (path.substringAfterLast('.', "").lowercase()) {
        "html", "htm" -> "text/html; charset=utf-8"
        "js"          -> "application/javascript; charset=utf-8"
        "css"         -> "text/css; charset=utf-8"
        "json"        -> "application/json; charset=utf-8"
        "svg"         -> "image/svg+xml"
        "ico"         -> "image/x-icon"
        else          -> "application/octet-stream"
    }

    private inner class RackScanSocket(handshake: IHTTPSession) : WebSocket(handshake) {
        override fun onOpen() {
            clients.add(this)
            // Send a snapshot immediately so a freshly-connected client doesn't
            // wait up to PUSH_INTERVAL_MS to see anything.
            try {
                val msg = "{\"telemetry\":${telemetry.snapshotJson()},\"logs\":${logs.snapshotJson()}}"
                send(msg)
            } catch (_: Throwable) {}
            Log.i(TAG, "client connected; total=${clients.size}")
        }
        override fun onClose(code: NanoWSD.WebSocketFrame.CloseCode?, reason: String?, initiatedByRemote: Boolean) {
            clients.remove(this); Log.i(TAG, "client closed; total=${clients.size}")
        }
        override fun onMessage(message: NanoWSD.WebSocketFrame) {
            val handler = commandHandler ?: return
            val text = try { message.textPayload } catch (_: Throwable) { return }
            if (text.isNullOrBlank()) return
            try {
                val obj = JSONObject(text)
                val cmd = obj.optString("cmd")
                if (cmd.isNotEmpty()) handler.onCommand(cmd, obj)
            } catch (t: Throwable) {
                Log.w(TAG, "bad command frame: $text (${t.message})")
            }
        }
        override fun onPong(pong: NanoWSD.WebSocketFrame) { /* no-op */ }
        override fun onException(exception: IOException) { clients.remove(this) }
    }

    companion object {
        private const val TAG = "RackScanDash"
        const val DEFAULT_PORT = 8082
        private const val PUSH_INTERVAL_MS = 200L
        private const val SOCKET_READ_TIMEOUT_MS = 0
    }
}
