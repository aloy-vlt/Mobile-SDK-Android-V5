package dji.sampleV5.aircraft.dashboard

import android.content.Context
import android.util.Log
import fi.iki.elonen.NanoHTTPD
import fi.iki.elonen.NanoWSD
import org.json.JSONObject
import java.io.IOException
import java.io.InputStream
import java.util.concurrent.CopyOnWriteArraySet

/**
 * Embedded HTTP + WebSocket server that serves the LAN dashboard and
 * accepts remote-control commands from connected clients.
 *
 * - GET /             -> assets/dashboard/index.html
 * - GET /<path>       -> assets/dashboard/<path>
 * - WS  /ws           -> server pushes binary JPEG frames + JSON telemetry;
 *                        clients push JSON command objects.
 *
 * Commands from clients arrive as text frames containing JSON of shape
 * `{"cmd": "<name>", ...args}`; they are forwarded to [commandHandler] which
 * is set by [DashboardServerVM] when it starts the server.
 */
class DashboardServer(
    port: Int,
    private val appContext: Context,
) : NanoWSD(port) {

    interface CommandHandler {
        fun onCommand(cmd: String, payload: JSONObject)
    }

    var commandHandler: CommandHandler? = null

    private val clients = CopyOnWriteArraySet<DashboardSocket>()

    @Volatile
    private var lastTelemetryJson: String? = null

    fun clientCount(): Int = clients.size

    override fun openWebSocket(handshake: IHTTPSession): WebSocket {
        return DashboardSocket(handshake)
    }

    override fun serveHttp(session: IHTTPSession): Response {
        val rawPath = session.uri.trimStart('/')
        val path = if (rawPath.isEmpty()) "index.html" else rawPath
        if (path.contains("..")) {
            return newFixedLengthResponse(Response.Status.FORBIDDEN, MIME_PLAINTEXT, "forbidden")
        }
        val assetPath = "dashboard/$path"
        return try {
            val stream: InputStream = appContext.assets.open(assetPath)
            val mime = mimeOf(path)
            newChunkedResponse(Response.Status.OK, mime, stream).apply {
                // Dashboard ships from the APK and changes with every install —
                // tell browsers never to cache it so updates are picked up
                // without users having to hard-refresh.
                addHeader("Cache-Control", "no-store, no-cache, must-revalidate")
                addHeader("Pragma", "no-cache")
                addHeader("Expires", "0")
            }
        } catch (e: IOException) {
            newFixedLengthResponse(Response.Status.NOT_FOUND, MIME_PLAINTEXT, "not found: $path")
        }
    }

    fun pushJpegFrame(jpeg: ByteArray) {
        for (c in clients) {
            try {
                c.send(jpeg)
            } catch (t: Throwable) {
                clients.remove(c)
            }
        }
    }

    fun pushTelemetry(json: String) {
        lastTelemetryJson = json
        for (c in clients) {
            try {
                c.send(json)
            } catch (t: Throwable) {
                clients.remove(c)
            }
        }
    }

    private fun mimeOf(path: String): String = when (path.substringAfterLast('.', "").lowercase()) {
        "html", "htm" -> "text/html; charset=utf-8"
        "js"          -> "application/javascript; charset=utf-8"
        "css"         -> "text/css; charset=utf-8"
        "json"        -> "application/json; charset=utf-8"
        "png"         -> "image/png"
        "jpg", "jpeg" -> "image/jpeg"
        "svg"         -> "image/svg+xml"
        "ico"         -> "image/x-icon"
        else          -> "application/octet-stream"
    }

    private inner class DashboardSocket(handshake: IHTTPSession) : WebSocket(handshake) {

        override fun onOpen() {
            clients.add(this)
            lastTelemetryJson?.let {
                try { send(it) } catch (_: Throwable) { /* ignore */ }
            }
            Log.i(TAG, "client connected; total=${clients.size}")
        }

        override fun onClose(code: WebSocketFrame.CloseCode?, reason: String?, initiatedByRemote: Boolean) {
            clients.remove(this)
            Log.i(TAG, "client closed; total=${clients.size}")
        }

        override fun onMessage(message: WebSocketFrame) {
            val handler = commandHandler ?: return
            val text = try { message.textPayload } catch (_: Throwable) { return }
            if (text.isNullOrBlank()) return
            try {
                val obj = JSONObject(text)
                val cmd = obj.optString("cmd")
                if (cmd.isNotEmpty()) {
                    handler.onCommand(cmd, obj)
                }
            } catch (e: Throwable) {
                Log.w(TAG, "bad command frame: $text")
            }
        }

        override fun onPong(pong: WebSocketFrame) { /* no-op */ }

        override fun onException(exception: IOException) {
            clients.remove(this)
        }
    }

    companion object {
        private const val TAG = "DashboardServer"
    }
}
