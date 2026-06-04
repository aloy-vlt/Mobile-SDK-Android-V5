package dji.sampleV5.aircraft.rackscan

import org.json.JSONArray
import org.json.JSONObject

/**
 * One step of a scripted mission. LEFT/RIGHT/HOVER are time-based; UP/DOWN are
 * distance-based (POSITION-mode altitude target). The dashboard's mission
 * builder produces these and the [MissionExecutor] consumes them.
 */
sealed class MissionStep {
    /** Approximate duration in seconds used for timeline rendering (UP/DOWN
     *  are FC-paced so this is a heuristic display value, not an enforcement). */
    abstract val displaySeconds: Float

    /** Human-readable single-word action label used by both UI and logs. */
    abstract val action: String

    data class Left(val seconds: Float) : MissionStep() {
        override val displaySeconds = seconds
        override val action = "LEFT"
    }

    data class Right(val seconds: Float) : MissionStep() {
        override val displaySeconds = seconds
        override val action = "RIGHT"
    }

    /** UP target = climbStartAlt + distanceM (POSITION mode). */
    data class Up(val distanceM: Float) : MissionStep() {
        // Rough display estimate: POSITION-mode climbs settle in about
        // (distance / 0.5 m/s) seconds for a 0.5 m/s effective FC ramp.
        override val displaySeconds = (distanceM / 0.5f).coerceAtLeast(0.5f)
        override val action = "UP"
    }

    /** DOWN target = climbStartAlt - distanceM, clamped to 0.3 m floor. */
    data class Down(val distanceM: Float) : MissionStep() {
        override val displaySeconds = (distanceM / 0.5f).coerceAtLeast(0.5f)
        override val action = "DOWN"
    }

    data class Hover(val seconds: Float) : MissionStep() {
        override val displaySeconds = seconds
        override val action = "HOVER"
    }

    /** Vision-positioning step: hold until the ArUco marker [markerId] is
     *  centered in the camera frame, then proceed. Used for GPS-denied indoor
     *  positioning — re-centering between sweeps corrects the lateral drift
     *  that open-loop (time-based) sweeps accumulate. The executor blocks on
     *  the host's centered signal (with a timeout); a host that doesn't do
     *  vision (the ArUco-Follow fragment) falls back to the no-op defaults on
     *  [MissionExecutor.Host], so this step only does real work where a vision
     *  pipeline implements it. */
    data class AlignAruco(val markerId: Int = 0) : MissionStep() {
        override val displaySeconds = 4f      // rough timeline-block size only
        override val action = "ALIGN"
    }

    /** Terminal step: the executor issues the land command and exits the
     *  mission after the drone reports motors off (or after LAND_TIMEOUT_MS).
     *  Any subsequent steps in the queue are skipped. */
    object Land : MissionStep() {
        override val displaySeconds = 6f      // rough timeline-block size only
        override val action = "LAND"
    }

    fun toJson(): JSONObject = JSONObject().apply {
        put("action", action)
        when (this@MissionStep) {
            is Left  -> put("seconds", seconds)
            is Right -> put("seconds", seconds)
            is Up    -> put("distance", distanceM)
            is Down  -> put("distance", distanceM)
            is Hover -> put("seconds", seconds)
            is AlignAruco -> put("markerId", markerId)
            Land     -> { /* no parameter */ }
        }
    }

    companion object {
        /** Parse a single step from JSON shape `{"action":"LEFT","seconds":3.0}`
         *  or `{"action":"UP","distance":0.5}`. Returns null on malformed input. */
        fun fromJson(o: JSONObject): MissionStep? = try {
            when (o.optString("action").uppercase()) {
                "LEFT"  -> Left(o.optDouble("seconds", 0.0).toFloat().coerceIn(0.1f, 30f))
                "RIGHT" -> Right(o.optDouble("seconds", 0.0).toFloat().coerceIn(0.1f, 30f))
                "UP"    -> Up(o.optDouble("distance", 0.0).toFloat().coerceIn(0.1f, 5f))
                "DOWN"  -> Down(o.optDouble("distance", 0.0).toFloat().coerceIn(0.1f, 5f))
                "HOVER" -> Hover(o.optDouble("seconds", 0.0).toFloat().coerceIn(0.1f, 60f))
                "ALIGN" -> AlignAruco(o.optInt("markerId", 0).coerceIn(0, 999))
                "LAND"  -> Land
                else    -> null
            }
        } catch (_: Throwable) { null }

        fun listFromJsonArray(arr: JSONArray): List<MissionStep> =
            buildList { for (i in 0 until arr.length()) fromJson(arr.optJSONObject(i) ?: continue)?.let(::add) }

        fun listToJsonArray(steps: List<MissionStep>): JSONArray =
            JSONArray().also { for (s in steps) it.put(s.toJson()) }
    }
}
