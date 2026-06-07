package dji.sampleV5.aircraft.rackscan

import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Runs a [MissionStep] list on a background thread, calling into the host
 * fragment's per-step hooks to drive the pump's state machine. Time-based
 * steps (LEFT / RIGHT / HOVER) sleep for the requested duration; distance-
 * based steps (UP / DOWN) wait for the altitude to reach target (with a
 * timeout).
 *
 * The host provides a [Host] callback so the executor doesn't need to know
 * about ArucoFollowFragment internals. All host calls happen on the mission
 * executor thread, so they MUST treat field writes as cross-thread:
 *   - State mutations should use @Volatile fields (already true in fragment).
 *   - UI updates should be marshalled via mainHandler inside the host impl.
 */
class MissionExecutor(private val host: Host, private val telemetry: RackScanTelemetry, private val logs: RackScanLogBuffer) {

    interface Host {
        /** Approximate current altitude in meters. */
        fun currentAltitudeM(): Double
        /** Start a LEFT sweep at the configured sweep speed (no marker advance). */
        fun missionStartLeft()
        /** Start a RIGHT sweep at the configured sweep speed. */
        fun missionStartRight()
        /** Start a POSITION-mode UP climb of `distanceM` from current altitude. */
        fun missionStartUp(distanceM: Float)
        /** Start a POSITION-mode DOWN climb of `distanceM` from current altitude. */
        fun missionStartDown(distanceM: Float)
        /** Park the drone (state -> IDLE, pump goes to hover throttle = 0). */
        fun missionStopAndHover()
        /** Issue the land command — disables VS, calls startLanding. */
        fun missionLand()
        /** Whether the FC reports motors on. Used to detect "landed". */
        fun motorsOn(): Boolean

        // ── ArUco align (vision positioning) — optional ──
        // Default no-ops so hosts without a vision pipeline (e.g. ArUco-Follow)
        // are unaffected; only hosts that implement marker centering override.
        /** Begin centering on ArUco marker [markerId]. */
        fun missionStartAlign(markerId: Int) {}
        /** True once the target marker is centered in the frame. Default true
         *  so a non-vision host treats an ALIGN step as an instant pass. */
        fun isCentered(): Boolean = true
    }

    private val exec = Executors.newSingleThreadExecutor { r ->
        Thread(r, "RackScanMission").apply { isDaemon = true }
    }
    private val running = AtomicBoolean(false)
    @Volatile private var requestStop: Boolean = false

    fun isRunning(): Boolean = running.get()

    /** Start running the given mission. Returns false if a mission is already running. */
    fun start(steps: List<MissionStep>, loop: Boolean = false): Boolean {
        if (!running.compareAndSet(false, true)) return false
        requestStop = false
        telemetry.missionRunning = true
        telemetry.missionLoop = loop
        telemetry.missionStepCount = steps.size
        telemetry.missionCurrentStep = -1
        telemetry.missionStepStartedMs = 0L
        telemetry.missionStepsJson = MissionStep.listToJsonArray(steps).toString()
        logs.i(TAG, "Mission start: ${steps.size} step(s)${if (loop) ", looping" else ""}")
        exec.execute { runMission(steps, loop) }
        return true
    }

    /** Request a graceful stop. The executor will park the drone in hover. */
    fun stop() {
        if (!running.get()) return
        requestStop = true
        logs.i(TAG, "Mission stop requested")
    }

    private fun runMission(steps: List<MissionStep>, loop: Boolean) {
        try {
            var lap = 0
            do {
                if (lap > 0) logs.i(TAG, "Mission loop iteration ${lap + 1}")
                for (idx in steps.indices) {
                    if (requestStop) return
                    val step = steps[idx]
                    telemetry.missionCurrentStep = idx
                    telemetry.missionStepStartedMs = System.currentTimeMillis()
                    logs.i(TAG, "Mission step ${idx + 1}/${steps.size}: ${describeStep(step)}")
                    runStep(step)
                    if (!requestStop) sleepInterruptible(300)
                }
                lap++
            } while (loop && !requestStop)
        } finally {
            host.missionStopAndHover()
            telemetry.missionRunning = false
            telemetry.missionLoop = false
            telemetry.missionCurrentStep = -1
            telemetry.missionStepStartedMs = 0L
            telemetry.missionStepCount = 0
            telemetry.missionStepsJson = "[]"
            running.set(false)
            requestStop = false
            logs.i(TAG, "Mission ended")
        }
    }

    private fun runStep(step: MissionStep) {
        when (step) {
            is MissionStep.Left  -> { host.missionStartLeft();  sleepSeconds(step.seconds); host.missionStopAndHover() }
            is MissionStep.Right -> { host.missionStartRight(); sleepSeconds(step.seconds); host.missionStopAndHover() }
            is MissionStep.Hover -> { host.missionStopAndHover(); sleepSeconds(step.seconds) }
            is MissionStep.Up    -> runVerticalStep(step.distanceM, up = true)
            is MissionStep.Down  -> runVerticalStep(step.distanceM, up = false)
            is MissionStep.AlignAruco -> runAlignStep(step.markerId)
            MissionStep.Land     -> runLandStep()
        }
    }

    /** Hold while the host centers the target ArUco marker. Proceeds once the
     *  marker has been reported centered continuously for [ALIGN_HOLD_MS], or
     *  after [ALIGN_TIMEOUT_MS] (logged) so a never-seen marker can't hang the
     *  mission. The host parks in hover afterwards. */
    private fun runAlignStep(markerId: Int) {
        host.missionStartAlign(markerId)
        val deadline = System.currentTimeMillis() + ALIGN_TIMEOUT_MS
        var centeredSince = 0L
        while (!requestStop && System.currentTimeMillis() < deadline) {
            if (host.isCentered()) {
                val now = System.currentTimeMillis()
                if (centeredSince == 0L) centeredSince = now
                else if (now - centeredSince >= ALIGN_HOLD_MS) { logs.i(TAG, "ArUco marker $markerId centered"); break }
            } else {
                centeredSince = 0L
            }
            sleepInterruptible(80)
        }
        if (System.currentTimeMillis() >= deadline) {
            logs.w(TAG, "Align timeout: marker $markerId not centered in ${ALIGN_TIMEOUT_MS / 1000}s — proceeding")
        }
        host.missionStopAndHover()
    }

    private fun runLandStep() {
        host.missionLand()
        // Wait until motors report off (drone fully landed) or timeout. After
        // a LAND step the mission is over — set requestStop so the outer loop
        // exits even when looping is on.
        val deadline = System.currentTimeMillis() + LAND_TIMEOUT_MS
        while (!requestStop && System.currentTimeMillis() < deadline && host.motorsOn()) {
            sleepInterruptible(200)
        }
        logs.i(TAG, "Land step complete (motorsOn=${host.motorsOn()})")
        requestStop = true
    }

    private fun runVerticalStep(distanceM: Float, up: Boolean) {
        val startAlt = host.currentAltitudeM()
        val targetAlt = if (up) startAlt + distanceM else (startAlt - distanceM).coerceAtLeast(0.30)
        if (up) host.missionStartUp(distanceM) else host.missionStartDown(distanceM)

        // POSITION-mode climbs are FC-paced. Poll altitude until we're inside
        // a 10 cm window of target, or until we time out.
        val deadline = System.currentTimeMillis() + VERTICAL_STEP_TIMEOUT_MS
        while (!requestStop && System.currentTimeMillis() < deadline) {
            val now = host.currentAltitudeM()
            if (kotlin.math.abs(now - targetAlt) < 0.10) break
            sleepInterruptible(100)
        }
        host.missionStopAndHover()
    }

    private fun sleepSeconds(seconds: Float) = sleepInterruptible((seconds * 1000).toLong())

    private fun sleepInterruptible(ms: Long) {
        val end = System.currentTimeMillis() + ms
        while (!requestStop) {
            val remaining = end - System.currentTimeMillis()
            if (remaining <= 0) return
            try { Thread.sleep(remaining.coerceAtMost(100)) } catch (_: InterruptedException) { return }
        }
    }

    private fun describeStep(s: MissionStep): String = when (s) {
        is MissionStep.Left  -> "LEFT ${s.seconds}s"
        is MissionStep.Right -> "RIGHT ${s.seconds}s"
        is MissionStep.Up    -> "UP ${s.distanceM}m"
        is MissionStep.Down  -> "DOWN ${s.distanceM}m"
        is MissionStep.Hover -> "HOVER ${s.seconds}s"
        is MissionStep.AlignAruco -> "ALIGN marker ${s.markerId}"
        MissionStep.Land     -> "LAND"
    }

    fun shutdown() {
        requestStop = true
        exec.shutdownNow()
    }

    companion object {
        private const val TAG = "Mission"
        private const val VERTICAL_STEP_TIMEOUT_MS = 15000L
        private const val LAND_TIMEOUT_MS          = 20000L
        private const val ALIGN_TIMEOUT_MS         = 20000L
        private const val ALIGN_HOLD_MS            = 1000L   // centered must hold this long
    }
}
