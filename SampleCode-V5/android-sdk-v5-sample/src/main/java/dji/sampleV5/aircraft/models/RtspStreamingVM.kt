package dji.sampleV5.aircraft.models

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.MutableLiveData
import dji.sampleV5.aircraft.rtsp.RtspStreamingService

/**
 * Thin ViewModel wrapper around [RtspStreamingService] so the fragment can
 * observe LiveData on the main thread. The service itself is app-scoped — it
 * survives fragment recreation and ViewModel destruction.
 */
class RtspStreamingVM(app: Application) : AndroidViewModel(app), RtspStreamingService.Listener {

    val state = MutableLiveData<RtspStreamingService.State>(RtspStreamingService.getState())
    val clientCount = MutableLiveData(RtspStreamingService.getClientCount())

    init {
        RtspStreamingService.addListener(this)
    }

    fun start(port: Int = RtspStreamingService.DEFAULT_PORT) {
        // RtspStreamingService.start binds a TCP server socket, which Android
        // forbids on the main thread (NetworkOnMainThreadException).
        Thread({ RtspStreamingService.start(port) }, "rtsp-vm-start").start()
    }

    fun stop() {
        Thread({ RtspStreamingService.stop() }, "rtsp-vm-stop").start()
    }

    override fun onStateChanged(state: RtspStreamingService.State) {
        this.state.postValue(state)
    }

    override fun onClientCountChanged(count: Int) {
        clientCount.postValue(count)
    }

    override fun onCleared() {
        super.onCleared()
        RtspStreamingService.removeListener(this)
    }
}
