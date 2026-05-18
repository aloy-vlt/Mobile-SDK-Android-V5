package dji.sampleV5.aircraft.pages

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import androidx.fragment.app.viewModels
import dji.sampleV5.aircraft.R
import dji.sampleV5.aircraft.models.RtspStreamingVM
import dji.sampleV5.aircraft.rtsp.RtspStreamingService

class RtspStreamingFragment : DJIFragment() {

    private val vm: RtspStreamingVM by viewModels()

    private lateinit var btnToggle: Button
    private lateinit var etPort: EditText
    private lateinit var tvStatus: TextView
    private lateinit var tvUrl: TextView
    private lateinit var tvStats: TextView

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?
    ): View? = inflater.inflate(R.layout.frag_rtsp_streaming, container, false)

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        btnToggle = view.findViewById(R.id.btn_rtsp_toggle)
        etPort = view.findViewById(R.id.et_rtsp_port)
        tvStatus = view.findViewById(R.id.tv_rtsp_status)
        tvUrl = view.findViewById(R.id.tv_rtsp_url)
        tvStats = view.findViewById(R.id.tv_rtsp_stats)

        etPort.setText(RtspStreamingService.DEFAULT_PORT.toString())

        btnToggle.setOnClickListener {
            when (vm.state.value) {
                is RtspStreamingService.State.Running -> vm.stop()
                else -> {
                    val port = etPort.text.toString().toIntOrNull()
                        ?: RtspStreamingService.DEFAULT_PORT
                    vm.start(port)
                }
            }
        }

        vm.state.observe(viewLifecycleOwner) { state ->
            when (state) {
                is RtspStreamingService.State.Stopped -> {
                    tvStatus.text = "Stopped"
                    tvUrl.text = ""
                    btnToggle.text = "Start RTSP"
                    etPort.isEnabled = true
                }
                is RtspStreamingService.State.Running -> {
                    tvStatus.text = "Running on port ${state.port}"
                    tvUrl.text = "Open in VLC / ffplay / OpenCV:\n${state.url}"
                    btnToggle.text = "Stop RTSP"
                    etPort.isEnabled = false
                }
                is RtspStreamingService.State.Error -> {
                    tvStatus.text = "Error: ${state.message}"
                    tvUrl.text = ""
                    btnToggle.text = "Start RTSP"
                    etPort.isEnabled = true
                }
                null -> Unit
            }
        }

        vm.clientCount.observe(viewLifecycleOwner) { c ->
            tvStats.text = "Clients: ${c ?: 0}"
        }
    }
}
