package dji.sampleV5.aircraft.pages

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.TextView
import androidx.fragment.app.viewModels
import dji.sampleV5.aircraft.R
import dji.sampleV5.aircraft.models.DashboardServerVM

class DashboardServerFragment : DJIFragment() {

    private val vm: DashboardServerVM by viewModels()

    private lateinit var btnToggle: Button
    private lateinit var tvStatus: TextView
    private lateinit var tvUrl: TextView
    private lateinit var tvStats: TextView

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?
    ): View? = inflater.inflate(R.layout.frag_dashboard_server, container, false)

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        btnToggle = view.findViewById(R.id.btn_dashboard_toggle)
        tvStatus = view.findViewById(R.id.tv_dashboard_status)
        tvUrl = view.findViewById(R.id.tv_dashboard_url)
        tvStats = view.findViewById(R.id.tv_dashboard_stats)

        btnToggle.setOnClickListener {
            when (val s = vm.serverState.value) {
                is DashboardServerVM.ServerState.Running -> vm.stop()
                else -> vm.start()
            }
        }

        vm.serverState.observe(viewLifecycleOwner) { state ->
            when (state) {
                is DashboardServerVM.ServerState.Stopped -> {
                    tvStatus.text = "Stopped"
                    tvUrl.text = ""
                    btnToggle.text = "Start Server"
                }
                is DashboardServerVM.ServerState.Running -> {
                    tvStatus.text = "Running on port ${state.port}"
                    tvUrl.text = "Open in browser: ${state.url}"
                    btnToggle.text = "Stop Server"
                }
                is DashboardServerVM.ServerState.Error -> {
                    tvStatus.text = "Error: ${state.message}"
                    tvUrl.text = ""
                    btnToggle.text = "Start Server"
                }
                null -> Unit
            }
        }

        vm.clientCount.observe(viewLifecycleOwner) { updateStats() }
        vm.fps.observe(viewLifecycleOwner) { updateStats() }
    }

    private fun updateStats() {
        val clients = vm.clientCount.value ?: 0
        val fps = vm.fps.value ?: 0
        tvStats.text = "Clients: $clients   |   FPS: $fps"
    }
}
