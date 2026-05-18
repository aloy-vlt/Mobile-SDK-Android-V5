package dji.sampleV5.aircraft.rtsp

import android.util.Log
import java.io.BufferedReader
import java.io.InputStreamReader
import java.net.DatagramSocket
import java.net.ServerSocket
import java.net.Socket
import java.util.UUID
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Minimal RTSP 1.0 server (RFC 2326) over TCP. Handles the subset needed by
 * common consumers (ffplay, OpenCV/FFmpeg, GStreamer, VLC):
 * OPTIONS / DESCRIBE / SETUP / PLAY / PAUSE / TEARDOWN / GET_PARAMETER.
 *
 * Transport: RTP/AVP unicast, both UDP and TCP-interleaved (RFC 2326 §10.12).
 * Default is UDP; the client opts into TCP via `Transport: RTP/AVP/TCP;
 * interleaved=N-M` in SETUP. Corvus / FFmpeg with `-rtsp_transport tcp`
 * uses the TCP path.
 *
 * Each client connection runs on its own thread; each PLAYing client gets a
 * [RtspSession] that pulls NALs from [MainCameraSource] and packetises them
 * via [RtpPacketizer]. Writes to the RTSP TCP socket — RTSP responses AND
 * any interleaved RTP frames — are serialized on the [Socket] monitor so the
 * two streams cannot corrupt each other.
 *
 * Stream URL: rtsp://<device-ip>:<port>/<streamPath>
 */
class RtspServer(
    private val port: Int,
    private val source: MainCameraSource,
    private val streamPath: String = "/fpv",
) {

    private val running = AtomicBoolean(false)
    private var serverSocket: ServerSocket? = null
    private val acceptExec = Executors.newSingleThreadExecutor { r ->
        Thread(r, "rtsp-accept").apply { isDaemon = true }
    }
    private val clientExec = Executors.newCachedThreadPool { r ->
        Thread(r, "rtsp-client").apply { isDaemon = true }
    }
    private val sessions = ConcurrentHashMap<String, RtspSession>()
    private val clientSockets = ConcurrentHashMap<String, Socket>()

    /** Called whenever a session is created or destroyed. */
    @Volatile var onClientCountChanged: ((Int) -> Unit)? = null

    fun start() {
        if (!running.compareAndSet(false, true)) return
        serverSocket = ServerSocket(port).apply { reuseAddress = true }
        acceptExec.execute(::acceptLoop)
        Log.i(TAG, "RTSP server listening on tcp/$port path $streamPath")
    }

    fun stop() {
        if (!running.compareAndSet(true, false)) return
        try { serverSocket?.close() } catch (_: Throwable) {}
        serverSocket = null
        for ((_, sess) in sessions) {
            try { sess.stop() } catch (_: Throwable) {}
        }
        sessions.clear()
        for ((_, sock) in clientSockets) {
            try { sock.close() } catch (_: Throwable) {}
        }
        clientSockets.clear()
        notifyClientCount()
    }

    fun isRunning(): Boolean = running.get()
    fun clientCount(): Int = sessions.size

    private fun acceptLoop() {
        val ss = serverSocket ?: return
        while (running.get()) {
            try {
                val client = ss.accept()
                clientExec.execute { handleClient(client) }
            } catch (t: Throwable) {
                if (!running.get()) break
                Log.w(TAG, "accept failed: ${t.message}")
            }
        }
    }

    private fun handleClient(socket: Socket) {
        val clientKey = "${socket.inetAddress.hostAddress}:${socket.port}"
        clientSockets[clientKey] = socket
        Log.i(TAG, "client connected: $clientKey")
        var ownedSessionId: String? = null
        try {
            val reader = BufferedReader(InputStreamReader(socket.getInputStream(), Charsets.US_ASCII))
            while (running.get() && !socket.isClosed) {
                val req = readRequest(reader) ?: break
                val sid = handleRequest(req, socket)
                if (sid != null) ownedSessionId = sid
            }
        } catch (t: Throwable) {
            Log.w(TAG, "client $clientKey loop error: ${t.message}")
        } finally {
            // If the TCP signaling channel drops without TEARDOWN, kill the
            // session anyway — otherwise the NAL consumer leaks.
            ownedSessionId?.let { sid ->
                sessions.remove(sid)?.let {
                    try { it.stop() } catch (_: Throwable) {}
                    notifyClientCount()
                }
            }
            clientSockets.remove(clientKey)
            try { socket.close() } catch (_: Throwable) {}
            Log.i(TAG, "client disconnected: $clientKey")
        }
    }

    private fun readRequest(reader: BufferedReader): RtspRequest? {
        val firstLine = reader.readLine() ?: return null
        if (firstLine.isBlank()) return null
        val parts = firstLine.trim().split(" ")
        if (parts.size < 3) return null
        val method = parts[0].uppercase()
        val uri = parts[1]
        val headers = LinkedHashMap<String, String>()
        while (true) {
            val line = reader.readLine() ?: break
            if (line.isBlank()) break
            val idx = line.indexOf(':')
            if (idx <= 0) continue
            val k = line.substring(0, idx).trim().lowercase()
            val v = line.substring(idx + 1).trim()
            headers[k] = v
        }
        // Drain any body (Content-Length) — none of the methods we serve carry
        // a request body, but be defensive against future SET_PARAMETER traffic.
        headers["content-length"]?.toIntOrNull()?.let { n ->
            if (n > 0) {
                val buf = CharArray(n)
                reader.read(buf, 0, n)
            }
        }
        return RtspRequest(method, uri, headers)
    }

    /** Returns the session id this request created (so the client thread can tear it down on disconnect). */
    private fun handleRequest(req: RtspRequest, socket: Socket): String? {
        val cseq = req.headers["cseq"] ?: "0"
        return when (req.method) {
            "OPTIONS"       -> { sendOptions(socket, cseq); null }
            "DESCRIBE"      -> { sendDescribe(socket, cseq, req.uri); null }
            "SETUP"         -> sendSetup(cseq, req.headers, socket)
            "PLAY"          -> { sendPlay(socket, cseq, req.headers); null }
            "PAUSE"         -> { sendPause(socket, cseq, req.headers); null }
            "TEARDOWN"      -> { sendTeardown(socket, cseq, req.headers); null }
            "GET_PARAMETER" -> { respond(socket, headers = linkedMapOf("CSeq" to cseq)); null }
            else            -> { respond(socket, 501, "Not Implemented", linkedMapOf("CSeq" to cseq)); null }
        }
    }

    private fun sendOptions(socket: Socket, cseq: String) {
        respond(socket, headers = linkedMapOf(
            "CSeq" to cseq,
            "Public" to "OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, TEARDOWN, GET_PARAMETER",
        ))
    }

    private fun sendDescribe(socket: Socket, cseq: String, uri: String) {
        val codec = source.codec
        if (codec == MainCameraSource.Codec.UNKNOWN) {
            respond(socket, 503, "Service Unavailable", linkedMapOf(
                "CSeq" to cseq,
                "Reason" to "waiting for first video frame from drone",
            ))
            return
        }
        val sdp = buildSdp(codec)
        respond(socket, headers = linkedMapOf(
            "CSeq" to cseq,
            "Content-Base" to "$uri/",
            "Content-Type" to "application/sdp",
        ), body = sdp)
    }

    private fun buildSdp(codec: MainCameraSource.Codec): String {
        val sb = StringBuilder()
        sb.append("v=0\r\n")
        sb.append("o=- 0 0 IN IP4 0.0.0.0\r\n")
        sb.append("s=DJI Drone Stream\r\n")
        sb.append("c=IN IP4 0.0.0.0\r\n")
        sb.append("t=0 0\r\n")
        sb.append("a=tool:dji-msdk-rtsp\r\n")
        sb.append("a=range:npt=0-\r\n")
        sb.append("m=video 0 RTP/AVP 96\r\n")
        if (codec == MainCameraSource.Codec.H265) {
            sb.append("a=rtpmap:96 H265/90000\r\n")
            val vps = source.vpsBase64()
            val sps = source.spsBase64()
            val pps = source.ppsBase64()
            if (vps != null && sps != null && pps != null) {
                sb.append("a=fmtp:96 sprop-vps=").append(vps)
                    .append("; sprop-sps=").append(sps)
                    .append("; sprop-pps=").append(pps)
                    .append("\r\n")
            }
        } else {
            sb.append("a=rtpmap:96 H264/90000\r\n")
            val sps = source.sps
            val spsB64 = source.spsBase64()
            val ppsB64 = source.ppsBase64()
            val profileLevelId = sps?.let { h264ProfileLevelId(it) } ?: "42e01f"
            sb.append("a=fmtp:96 packetization-mode=1; profile-level-id=").append(profileLevelId)
            if (spsB64 != null && ppsB64 != null) {
                sb.append("; sprop-parameter-sets=").append(spsB64).append(",").append(ppsB64)
            }
            sb.append("\r\n")
        }
        sb.append("a=control:track0\r\n")
        return sb.toString()
    }

    private fun h264ProfileLevelId(sps: ByteArray): String {
        if (sps.size < 4) return "42e01f"
        return String.format(
            "%02x%02x%02x",
            sps[1].toInt() and 0xFF,
            sps[2].toInt() and 0xFF,
            sps[3].toInt() and 0xFF,
        )
    }

    private fun sendSetup(
        cseq: String, headers: Map<String, String>, socket: Socket
    ): String? {
        val transport = headers["transport"] ?: run {
            respond(socket, 461, "Unsupported Transport", linkedMapOf("CSeq" to cseq))
            return null
        }
        if (!transport.contains("RTP/AVP", ignoreCase = true)) {
            respond(socket, 461, "Unsupported Transport", linkedMapOf("CSeq" to cseq))
            return null
        }

        val isTcpInterleaved = transport.contains("RTP/AVP/TCP", ignoreCase = true) ||
            transport.contains("interleaved", ignoreCase = true)

        val sessionId = UUID.randomUUID().toString().replace("-", "").take(16)

        val (sender, responseTransport) = if (isTcpInterleaved) {
            buildTcpSetup(transport, socket, sessionId) ?: run {
                respond(socket, 461, "Unsupported Transport", linkedMapOf("CSeq" to cseq))
                return null
            }
        } else {
            buildUdpSetup(transport, socket) ?: run {
                respond(socket, 461, "Unsupported Transport", linkedMapOf("CSeq" to cseq))
                return null
            }
        }

        val session = RtspSession(
            id = sessionId,
            source = source,
            sender = sender,
            peerLabel = if (isTcpInterleaved) {
                "tcp ${socket.inetAddress.hostAddress}:${socket.port}"
            } else {
                "${socket.inetAddress.hostAddress}:${parseClientPort(transport)?.first ?: 0}"
            },
        )
        sessions[sessionId] = session
        notifyClientCount()

        respond(socket, headers = linkedMapOf(
            "CSeq" to cseq,
            "Transport" to responseTransport,
            "Session" to "$sessionId;timeout=60",
        ))
        return sessionId
    }

    /** Returns (sender, response Transport header) for UDP, or null if the client's spec is unusable. */
    private fun buildUdpSetup(transport: String, socket: Socket): Pair<RtpSender, String>? {
        val (clientRtp, clientRtcp) = parseClientPort(transport) ?: return null
        val rtpSocket = try {
            DatagramSocket().apply {
                // DSCP EF — best-effort hint for low-latency video on
                // hardware that honours it.
                try { trafficClass = 0xB8 } catch (_: Throwable) {}
                // A 1080p I-frame can be 30-60 FU-A packets shipped back-to-back
                // from the MSDK callback thread. The default ~200 KB UDP send
                // buffer overflows on those bursts and the kernel drops packets
                // silently, which presents downstream as ffmpeg's "RTP: missed
                // N packets" + I-frame error concealment. 1 MB absorbs a single
                // GOP comfortably.
                try { sendBufferSize = 1 shl 20 } catch (_: Throwable) {}
                // Pre-resolve the route so each send() skips the per-packet
                // socket-table lookup. Reduces scheduling jitter during the
                // FU-A burst.
                try { connect(socket.inetAddress, clientRtp) } catch (_: Throwable) {}
            }
        } catch (_: Throwable) {
            return null
        }
        val serverRtp = rtpSocket.localPort
        val serverRtcp = serverRtp + 1
        val sender = UdpRtpSender(
            socket = rtpSocket,
            destAddr = socket.inetAddress,
            destPort = clientRtp,
        )
        val responseTransport =
            "RTP/AVP;unicast;client_port=$clientRtp-$clientRtcp;server_port=$serverRtp-$serverRtcp"
        return sender to responseTransport
    }

    /** Returns (sender, response Transport header) for TCP-interleaved, or null on bad spec. */
    private fun buildTcpSetup(transport: String, socket: Socket, sessionId: String): Pair<RtpSender, String>? {
        val (rtpCh, rtcpCh) = parseInterleavedChannels(transport) ?: return null
        // TCP_NODELAY: small RTP packets shouldn't pool in Nagle's send window;
        // we already coalesce at the application layer (one write per packet).
        try { socket.tcpNoDelay = true } catch (_: Throwable) {}
        // Bigger send buffer absorbs FU-A bursts the same way as the UDP path.
        try { socket.sendBufferSize = 1 shl 20 } catch (_: Throwable) {}
        val sender = TcpInterleavedRtpSender(
            out = socket.getOutputStream(),
            writeMonitor = socket,
            rtpChannel = rtpCh,
            sessionId = sessionId,
        )
        val responseTransport = "RTP/AVP/TCP;unicast;interleaved=$rtpCh-$rtcpCh"
        return sender to responseTransport
    }

    private fun parseClientPort(transport: String): Pair<Int, Int>? {
        val token = transport.split(";")
            .firstOrNull { it.trim().startsWith("client_port", ignoreCase = true) }
            ?: return null
        val v = token.substringAfter("=", "").trim()
        val parts = v.split("-")
        if (parts.isEmpty()) return null
        val a = parts[0].toIntOrNull() ?: return null
        val b = parts.getOrNull(1)?.toIntOrNull() ?: (a + 1)
        return a to b
    }

    private fun parseInterleavedChannels(transport: String): Pair<Int, Int>? {
        val token = transport.split(";")
            .firstOrNull { it.trim().startsWith("interleaved", ignoreCase = true) }
            ?: return 0 to 1  // RFC default when interleaved is named without channels
        val v = token.substringAfter("=", "").trim()
        if (v.isEmpty()) return 0 to 1
        val parts = v.split("-")
        val a = parts[0].toIntOrNull() ?: return null
        val b = parts.getOrNull(1)?.toIntOrNull() ?: (a + 1)
        return a to b
    }

    private fun sendPlay(socket: Socket, cseq: String, headers: Map<String, String>) {
        val sess = sessionFor(headers) ?: run {
            respond(socket, 454, "Session Not Found", linkedMapOf("CSeq" to cseq)); return
        }
        // Send the PLAY response BEFORE start() so the response can't get
        // interleaved between RTP frames on the TCP path (start() may begin
        // queueing NALs immediately).
        respond(socket, headers = linkedMapOf(
            "CSeq" to cseq,
            "Session" to sess.id,
            "Range" to "npt=0.000-",
            "RTP-Info" to "url=rtsp://0.0.0.0$streamPath/track0;seq=0;rtptime=0",
        ))
        sess.start()
    }

    private fun sendPause(socket: Socket, cseq: String, headers: Map<String, String>) {
        val sess = sessionFor(headers) ?: run {
            respond(socket, 454, "Session Not Found", linkedMapOf("CSeq" to cseq)); return
        }
        sess.stop()
        respond(socket, headers = linkedMapOf("CSeq" to cseq, "Session" to sess.id))
    }

    private fun sendTeardown(socket: Socket, cseq: String, headers: Map<String, String>) {
        val sess = sessionFor(headers) ?: run {
            respond(socket, 454, "Session Not Found", linkedMapOf("CSeq" to cseq)); return
        }
        sess.stop()
        sessions.remove(sess.id)
        notifyClientCount()
        respond(socket, headers = linkedMapOf("CSeq" to cseq, "Session" to sess.id))
    }

    private fun sessionFor(headers: Map<String, String>): RtspSession? {
        val sid = headers["session"]?.split(";")?.firstOrNull()?.trim() ?: return null
        return sessions[sid]
    }

    private fun respond(
        socket: Socket,
        code: Int = 200,
        reason: String = "OK",
        headers: LinkedHashMap<String, String>,
        body: String? = null,
    ) {
        val finalHeaders = if (body != null) {
            val bytes = body.toByteArray(Charsets.US_ASCII).size
            LinkedHashMap(headers).apply { put("Content-Length", bytes.toString()) }
        } else headers
        val sb = StringBuilder()
        sb.append("RTSP/1.0 ").append(code).append(' ').append(reason).append("\r\n")
        for ((k, v) in finalHeaders) sb.append(k).append(": ").append(v).append("\r\n")
        sb.append("\r\n")
        if (body != null) sb.append(body)
        val bytes = sb.toString().toByteArray(Charsets.US_ASCII)
        // Lock on the Socket monitor — TcpInterleavedRtpSender uses the same
        // monitor when writing `$`-framed RTP packets. Bypass the PrintWriter
        // so we don't have to fight its hidden buffering with binary writes.
        try {
            synchronized(socket) {
                val out = socket.getOutputStream()
                out.write(bytes)
                out.flush()
            }
        } catch (t: Throwable) {
            Log.w(TAG, "respond failed: ${t.message}")
        }
    }

    private fun notifyClientCount() {
        try { onClientCountChanged?.invoke(sessions.size) } catch (_: Throwable) {}
    }

    private data class RtspRequest(
        val method: String,
        val uri: String,
        val headers: Map<String, String>,
    )

    companion object {
        private const val TAG = "RtspServer"
    }
}
