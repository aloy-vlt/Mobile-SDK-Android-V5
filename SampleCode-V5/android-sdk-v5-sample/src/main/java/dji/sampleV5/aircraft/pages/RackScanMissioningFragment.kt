package dji.sampleV5.aircraft.pages

import android.content.ClipData
import android.content.ClipboardManager
import android.content.Context
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.TextView
import androidx.fragment.app.viewModels
import dji.sampleV5.aircraft.R
import dji.sampleV5.aircraft.models.RackScanMissionVM
import dji.sampleV5.aircraft.rtsp.RtspStreamingService
import dji.sampleV5.aircraft.util.ToastUtils

/**
 * Operator-facing control screen for the Rack Scan Missioning module.
 *
 * Intentionally thin: it starts/stops the dashboard server and the RTSP feed
 * and surfaces the URLs operators need to open on their laptop/tablet. The
 * mission itself is assigned and monitored from the web dashboard (the URL
 * shown here); all heavy lifting lives in [RackScanMissionVM].
 */
class RackScanMissioningFragment : DJIFragment() {

    private val vm: RackScanMissionVM by viewModels()

    private lateinit var btnDashToggle: Button
    private lateinit var tvDashStatus: TextView
    private lateinit var tvDashUrl: TextView
    private lateinit var tvStats: TextView
    private lateinit var btnRtspToggle: Button
    private lateinit var tvRtspStatus: TextView
    private lateinit var tvRtspUrl: TextView

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?
    ): View? = inflater.inflate(R.layout.frag_rack_scan_missioning, container, false)

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        btnDashToggle = view.findViewById(R.id.btn_rm_dashboard_toggle)
        tvDashStatus = view.findViewById(R.id.tv_rm_dashboard_status)
        tvDashUrl = view.findViewById(R.id.tv_rm_dashboard_url)
        tvStats = view.findViewById(R.id.tv_rm_stats)
        btnRtspToggle = view.findViewById(R.id.btn_rm_rtsp_toggle)
        tvRtspStatus = view.findViewById(R.id.tv_rm_rtsp_status)
        tvRtspUrl = view.findViewById(R.id.tv_rm_rtsp_url)

        btnDashToggle.setOnClickListener {
            when (vm.serverState.value) {
                is RackScanMissionVM.ServerState.Running -> vm.stop()
                else -> vm.start()
            }
        }

        btnRtspToggle.setOnClickListener {
            if (RtspStreamingService.isRunning()) {
                vm.onCommand("stopRtsp", org.json.JSONObject())
            } else {
                vm.onCommand("startRtsp", org.json.JSONObject())
            }
        }

        // Tap a URL to copy it — saves typing a phone IP into a laptop browser.
        tvDashUrl.setOnClickListener { copyToClipboard(tvDashUrl.text?.toString()) }
        tvRtspUrl.setOnClickListener { copyToClipboard(tvRtspUrl.text?.toString()) }

        vm.serverState.observe(viewLifecycleOwner) { state ->
            when (state) {
                is RackScanMissionVM.ServerState.Stopped -> {
                    tvDashStatus.text = "Dashboard: stopped"
                    tvDashUrl.text = ""
                    btnDashToggle.text = "Start Dashboard"
                }
                is RackScanMissionVM.ServerState.Running -> {
                    tvDashStatus.text = "Dashboard: running on port ${state.port}"
                    tvDashUrl.text = state.url
                    btnDashToggle.text = "Stop Dashboard"
                }
                is RackScanMissionVM.ServerState.Error -> {
                    tvDashStatus.text = "Dashboard error: ${state.message}"
                    tvDashUrl.text = ""
                    btnDashToggle.text = "Start Dashboard"
                }
                null -> Unit
            }
        }

        vm.clientCount.observe(viewLifecycleOwner) { updateStats() }
        vm.fps.observe(viewLifecycleOwner) { updateStats() }

        vm.rtspRunning.observe(viewLifecycleOwner) { running ->
            btnRtspToggle.text = if (running == true) "Stop RTSP Stream" else "Start RTSP Stream"
            tvRtspStatus.text = if (running == true) "RTSP: streaming" else "RTSP: stopped"
        }
        vm.rtspUrl.observe(viewLifecycleOwner) { url ->
            tvRtspUrl.text = url ?: ""
        }
    }

    private fun updateStats() {
        val clients = vm.clientCount.value ?: 0
        val fps = vm.fps.value ?: 0
        tvStats.text = "Dashboard clients: $clients    Feed: $fps fps"
    }

    private fun copyToClipboard(text: String?) {
        if (text.isNullOrBlank()) return
        val cm = context?.getSystemService(Context.CLIPBOARD_SERVICE) as? ClipboardManager ?: return
        cm.setPrimaryClip(ClipData.newPlainText("url", text))
        ToastUtils.showToast("Copied: $text")
    }
}
