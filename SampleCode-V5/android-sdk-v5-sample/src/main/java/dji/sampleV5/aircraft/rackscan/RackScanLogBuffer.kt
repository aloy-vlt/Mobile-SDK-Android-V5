package dji.sampleV5.aircraft.rackscan

import android.util.Log
import java.util.concurrent.ConcurrentLinkedDeque

/**
 * Ring-buffer of recent log entries from ArucoFollowFragment, surfaced to the
 * dashboard so the operator can inspect what the fragment was doing at a
 * specific moment in time.
 *
 * Append from any thread; the dashboard server snapshots periodically. The
 * deque is bounded to [maxEntries] to keep memory and per-snapshot JSON size
 * predictable.
 */
class RackScanLogBuffer(private val maxEntries: Int = 200) {

    enum class Level { I, W, E }

    data class Entry(val timestampMs: Long, val level: Level, val tag: String, val msg: String)

    private val entries = ConcurrentLinkedDeque<Entry>()

    fun i(tag: String, msg: String) { add(Level.I, tag, msg); Log.i(tag, msg) }
    fun w(tag: String, msg: String) { add(Level.W, tag, msg); Log.w(tag, msg) }
    fun e(tag: String, msg: String) { add(Level.E, tag, msg); Log.e(tag, msg) }

    private fun add(level: Level, tag: String, msg: String) {
        entries.addLast(Entry(System.currentTimeMillis(), level, tag, msg))
        while (entries.size > maxEntries) entries.pollFirst()
    }

    /**
     * Emit the current entries as a JSON array. Each entry is
     * `{"t":<ms>,"l":"I|W|E","tag":"...","msg":"..."}`.
     */
    fun snapshotJson(): String = buildString(2048) {
        append('[')
        var first = true
        for (e in entries) {
            if (!first) append(',') else first = false
            append('{')
            append("\"t\":").append(e.timestampMs).append(',')
            append("\"l\":\"").append(e.level.name).append("\",")
            append("\"tag\":\"").append(escape(e.tag)).append("\",")
            append("\"msg\":\"").append(escape(e.msg)).append("\"")
            append('}')
        }
        append(']')
    }

    private fun escape(s: String): String =
        s.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", "\\n").replace("\r", "\\r")
}
