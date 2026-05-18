package dji.sampleV5.aircraft.rackscan

/**
 * Live telemetry from ArucoFollowFragment that the dashboard streams to browsers.
 *
 * Single mutable instance written by frame/pump threads, read by the dashboard
 * server's push loop. All scalar fields are @Volatile; collections are
 * snapshotted via [snapshotJson] to avoid concurrent-modification races.
 */
class RackScanTelemetry {

    // ── Scan state machine ──
    @Volatile var stateName: String     = "IDLE"
    @Volatile var currentTargetId: Int  = 0
    @Volatile var currentLevel: Int     = 0
    @Volatile var numLevels: Int        = 0
    @Volatile var totalMarkers: Int     = 0
    @Volatile var isTracking: Boolean   = false
    @Volatile var isVSEnabled: Boolean  = false

    // ── Marker detection (last frame) ──
    @Volatile var detected: Boolean    = false
    @Volatile var offsetX: Float       = 0f
    @Volatile var markerSize: Float    = 0f
    @Volatile var visibleIdsCsv: String = ""

    // ── Pump output (last tick) ──
    @Volatile var rollPitchControlMode: String = "ANGLE"
    @Volatile var verticalControlMode: String  = "VELOCITY"
    @Volatile var rollDeg: Double              = 0.0     // ANGLE mode: degrees
    @Volatile var pitchMps: Double             = 0.0     // VELOCITY mode (sweep): m/s
    @Volatile var verticalThrottleMps: Double  = 0.0     // unit depends on verticalControlMode
    @Volatile var pumpTicks: Long              = 0L
    @Volatile var inHoverLock: Boolean         = true
    @Volatile var inKick: Boolean              = false
    @Volatile var inVerticalKick: Boolean      = false

    // ── PD gains and speed knobs ──
    @Volatile var pGain: Float          = 0f
    @Volatile var dGain: Float          = 0f
    @Volatile var sweepSpeedMps: Float  = 0f
    @Volatile var climbSpeedMps: Float  = 0f

    // ── Flight state ──
    @Volatile var altitudeM: Double         = 0.0
    @Volatile var heightLimitM: Int         = -1     // -1 = unread/unknown
    @Volatile var oaMode: String            = "UNKNOWN"
    @Volatile var nearestObstacleM: Float   = -1f
    @Volatile var lastMarkerAgeSec: Float   = -1f

    // ── Aircraft actual motion (read back from FC, NOT commanded) ──
    @Volatile var motorsOn: Boolean         = false
    @Volatile var velocityX: Double         = 0.0   // m/s, NEU frame
    @Volatile var velocityY: Double         = 0.0
    @Volatile var velocityZ: Double         = 0.0   // positive = up (or down, depending on SDK)

    // VPS (vision positioning) — fragment toggles it off during CLIMBING_UP
    // to release the FC's downward-sensor altitude hold that was filtering our
    // VELOCITY commands. Restored when leaving the climb state.
    @Volatile var vpsDisabledByUs: Boolean  = false

    // ── Mission scripting (dashboard-driven) ──
    @Volatile var missionRunning: Boolean      = false
    @Volatile var missionLoop: Boolean         = false
    @Volatile var missionStepCount: Int        = 0
    @Volatile var missionCurrentStep: Int      = -1   // 0-based, -1 = none
    @Volatile var missionStepStartedMs: Long   = 0L
    // JSON array of the currently-loaded mission (so the dashboard can render
    // the timeline even after a reconnect). Written by MissionExecutor.
    @Volatile var missionStepsJson: String     = "[]"

    // ── Counters / timestamps (for the dashboard "last activity" widget) ──
    @Volatile var serverPushCount: Long     = 0L
    @Volatile var lastUpdateMs: Long        = 0L

    /**
     * Serialize to a JSON object. Caller adds the wrapping `{ "telemetry": ..., "logs": ... }`.
     */
    fun snapshotJson(): String = buildString(640) {
        append('{')
        append("\"stateName\":\"").append(stateName).append("\",")
        append("\"currentTargetId\":").append(currentTargetId).append(',')
        append("\"currentLevel\":").append(currentLevel).append(',')
        append("\"numLevels\":").append(numLevels).append(',')
        append("\"totalMarkers\":").append(totalMarkers).append(',')
        append("\"isTracking\":").append(isTracking).append(',')
        append("\"isVSEnabled\":").append(isVSEnabled).append(',')
        append("\"detected\":").append(detected).append(',')
        append("\"offsetX\":").append(offsetX).append(',')
        append("\"markerSize\":").append(markerSize).append(',')
        append("\"visibleIds\":\"").append(visibleIdsCsv).append("\",")
        append("\"rollPitchControlMode\":\"").append(rollPitchControlMode).append("\",")
        append("\"verticalControlMode\":\"").append(verticalControlMode).append("\",")
        append("\"rollDeg\":").append(rollDeg).append(',')
        append("\"pitchMps\":").append(pitchMps).append(',')
        append("\"verticalThrottleMps\":").append(verticalThrottleMps).append(',')
        append("\"pumpTicks\":").append(pumpTicks).append(',')
        append("\"inHoverLock\":").append(inHoverLock).append(',')
        append("\"inKick\":").append(inKick).append(',')
        append("\"inVerticalKick\":").append(inVerticalKick).append(',')
        append("\"pGain\":").append(pGain).append(',')
        append("\"dGain\":").append(dGain).append(',')
        append("\"sweepSpeedMps\":").append(sweepSpeedMps).append(',')
        append("\"climbSpeedMps\":").append(climbSpeedMps).append(',')
        append("\"altitudeM\":").append(altitudeM).append(',')
        append("\"heightLimitM\":").append(heightLimitM).append(',')
        append("\"oaMode\":\"").append(oaMode).append("\",")
        append("\"nearestObstacleM\":").append(nearestObstacleM).append(',')
        append("\"lastMarkerAgeSec\":").append(lastMarkerAgeSec).append(',')
        append("\"motorsOn\":").append(motorsOn).append(',')
        append("\"velocityX\":").append(velocityX).append(',')
        append("\"velocityY\":").append(velocityY).append(',')
        append("\"velocityZ\":").append(velocityZ).append(',')
        append("\"vpsDisabledByUs\":").append(vpsDisabledByUs).append(',')
        append("\"missionRunning\":").append(missionRunning).append(',')
        append("\"missionLoop\":").append(missionLoop).append(',')
        append("\"missionStepCount\":").append(missionStepCount).append(',')
        append("\"missionCurrentStep\":").append(missionCurrentStep).append(',')
        append("\"missionStepStartedMs\":").append(missionStepStartedMs).append(',')
        append("\"missionStepsJson\":").append(missionStepsJson).append(',')
        append("\"serverPushCount\":").append(serverPushCount).append(',')
        append("\"lastUpdateMs\":").append(lastUpdateMs)
        append('}')
    }
}
