package dji.sampleV5.aircraft.pages

import android.graphics.Bitmap
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.SeekBar
import android.widget.TextView
import android.widget.ImageView
import androidx.fragment.app.activityViewModels
import dji.sampleV5.aircraft.R
import dji.sampleV5.aircraft.models.BasicAircraftControlVM
import dji.sampleV5.aircraft.models.VirtualStickVM
import dji.sampleV5.aircraft.util.ToastUtils
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.aircraft.virtualstick.Stick
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import org.opencv.android.OpenCVLoader
import org.opencv.android.Utils
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import org.opencv.objdetect.ArucoDetector
import org.opencv.objdetect.DetectorParameters
import org.opencv.objdetect.Objdetect
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.abs

class ArucoFollowFragment : DJIFragment() {

    companion object {
        private const val TAG = "ArucoFollow"
        private const val TARGET_MARKER_ID = 0

        // PD Controller tuning — start LOW, increase gradually
        private const val DEAD_ZONE = 0.05f       // Ignore tiny offsets
        private const val MAX_STICK_RATIO = 0.15f  // Max 15% stick = gentle movement

        // Center hold for auto-land
        private const val CENTER_HOLD_TIME_MS = 3000L
    }

    private val basicAircraftControlVM: BasicAircraftControlVM by activityViewModels()
    private val virtualStickVM: VirtualStickVM by activityViewModels()

    private lateinit var tvStatus: TextView
    private lateinit var tvOffset: TextView
    private lateinit var tvGain: TextView
    private lateinit var tvDGain: TextView
    private lateinit var btnTakeOff: Button
    private lateinit var btnLand: Button
    private lateinit var btnEnableVS: Button
    private lateinit var btnStartTrack: Button
    private lateinit var seekGain: SeekBar
    private lateinit var seekDGain: SeekBar
    private lateinit var imgPreview: ImageView

    private val isTracking = AtomicBoolean(false)
    private var pGain = 0.15f       // Proportional gain — how hard it corrects offset
    private var dGain = 0.10f       // Derivative gain — dampens oscillation
    private var prevOffsetX = 0f    // Previous frame offset for derivative
    private var centerStartTime = 0L
    private var isCentered = false

    private lateinit var arucoDetector: ArucoDetector

    private lateinit var processingThread: HandlerThread
    private lateinit var processingHandler: Handler
    private val isProcessing = AtomicBoolean(false)

    private val frameListener = object : ICameraStreamManager.CameraFrameListener {
        override fun onFrame(
            frameData: ByteArray,
            offset: Int,
            length: Int,
            width: Int,
            height: Int,
            format: ICameraStreamManager.FrameFormat
        ) {
            if (!isTracking.get()) return
            if (isProcessing.get()) return

            val data = ByteArray(length)
            System.arraycopy(frameData, offset, data, 0, length)

            processingHandler.post {
                processFrame(data, width, height)
            }
        }
    }

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?
    ): View? {
        return inflater.inflate(R.layout.frag_aruco_follow, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        if (!OpenCVLoader.initLocal()) {
            Log.e(TAG, "OpenCV init failed!")
            ToastUtils.showToast("OpenCV init failed!")
            return
        }
        Log.i(TAG, "OpenCV loaded successfully")

        val dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50)
        val parameters = DetectorParameters()
        arucoDetector = ArucoDetector(dictionary, parameters)

        processingThread = HandlerThread("ArucoProcessing").also { it.start() }
        processingHandler = Handler(processingThread.looper)

        initUI(view)
        setupCameraStream()
    }

    private fun initUI(view: View) {
        tvStatus = view.findViewById(R.id.tv_status)
        tvOffset = view.findViewById(R.id.tv_offset)
        tvGain = view.findViewById(R.id.tv_gain)
        tvDGain = view.findViewById(R.id.tv_dgain)
        btnTakeOff = view.findViewById(R.id.btn_takeoff)
        btnLand = view.findViewById(R.id.btn_land)
        btnEnableVS = view.findViewById(R.id.btn_enable_vs)
        btnStartTrack = view.findViewById(R.id.btn_start_track)
        seekGain = view.findViewById(R.id.seek_gain)
        seekDGain = view.findViewById(R.id.seek_dgain)
        imgPreview = view.findViewById(R.id.img_preview)

        updateGainText()

        btnTakeOff.setOnClickListener {
            basicAircraftControlVM.startTakeOff(object :
                CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) {
                    mainHandler.post { tvStatus.text = "Take off success - hovering" }
                    ToastUtils.showToast("Take off success")
                }
                override fun onFailure(error: IDJIError) {
                    ToastUtils.showToast("Take off failed: $error")
                }
            })
        }

        btnLand.setOnClickListener {
            stopTracking()
            basicAircraftControlVM.startLanding(object :
                CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) {
                    mainHandler.post { tvStatus.text = "Landing..." }
                    ToastUtils.showToast("Landing started")
                }
                override fun onFailure(error: IDJIError) {
                    ToastUtils.showToast("Landing failed: $error")
                }
            })
        }

        btnEnableVS.setOnClickListener {
            virtualStickVM.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    mainHandler.post {
                        tvStatus.text = "Virtual Stick ENABLED"
                        btnEnableVS.text = "VS Enabled"
                    }
                    ToastUtils.showToast("Virtual Stick enabled")
                }
                override fun onFailure(error: IDJIError) {
                    ToastUtils.showToast("VS enable failed: $error")
                }
            })
        }

        btnStartTrack.setOnClickListener {
            if (isTracking.get()) {
                stopTracking()
            } else {
                startTracking()
            }
        }

        // P-Gain slider: 0.05 to 0.50 (gentle range)
        seekGain.max = 45
        seekGain.progress = ((pGain - 0.05f) * 100).toInt()
        seekGain.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                pGain = 0.05f + progress / 100f
                updateGainText()
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        // D-Gain slider: 0.0 to 0.30
        seekDGain.max = 30
        seekDGain.progress = (dGain * 100).toInt()
        seekDGain.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                dGain = progress / 100f
                updateGainText()
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })
    }

    private fun setupCameraStream() {
        try {
            MediaDataCenter.getInstance().cameraStreamManager.addFrameListener(
                ComponentIndexType.LEFT_OR_MAIN,
                ICameraStreamManager.FrameFormat.NV21,
                frameListener
            )
            Log.i(TAG, "Camera stream listener registered")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to setup camera stream: ${e.message}")
            mainHandler.post { tvStatus.text = "Camera setup failed - connect drone first" }
        }
    }

    private fun processFrame(nv21Data: ByteArray, width: Int, height: Int) {
        if (!isTracking.get()) return
        isProcessing.set(true)

        try {
            val yuvMat = Mat(height + height / 2, width, CvType.CV_8UC1)
            yuvMat.put(0, 0, nv21Data)
            val bgrMat = Mat()
            Imgproc.cvtColor(yuvMat, bgrMat, Imgproc.COLOR_YUV2BGR_NV21)
            yuvMat.release()

            val corners = mutableListOf<Mat>()
            val ids = Mat()
            arucoDetector.detectMarkers(bgrMat, corners, ids)

            var offsetX = 0f
            var detected = false
            var markerSize = 0f

            if (ids.rows() > 0) {
                for (i in 0 until ids.rows()) {
                    if (ids[i, 0][0].toInt() == TARGET_MARKER_ID) {
                        detected = true
                        val mc = corners[i]
                        val cx = (0 until 4).map { mc[0, it][0] }.average().toFloat()

                        val p0x = mc[0, 0][0]; val p0y = mc[0, 0][1]
                        val p2x = mc[0, 2][0]; val p2y = mc[0, 2][1]
                        markerSize = Math.sqrt(
                            (p2x - p0x) * (p2x - p0x) + (p2y - p0y) * (p2y - p0y)
                        ).toFloat()

                        offsetX = (cx - width / 2f) / (width / 2f)
                        break
                    }
                }
            }

            if (detected) {
                sendRollCommand(offsetX)
                checkCentered(offsetX)
            } else {
                // No marker — hover
                virtualStickVM.setRightPosition(0, 0)
                virtualStickVM.setLeftPosition(0, 0)
                centerStartTime = 0L
                isCentered = false
                prevOffsetX = 0f
            }

            // Preview bitmap
            val rgbMat = Mat()
            Imgproc.cvtColor(bgrMat, rgbMat, Imgproc.COLOR_BGR2RGB)
            val bitmap = Bitmap.createBitmap(rgbMat.cols(), rgbMat.rows(), Bitmap.Config.ARGB_8888)
            Utils.matToBitmap(rgbMat, bitmap)

            val finalOffsetX = offsetX
            val finalDetected = detected
            val finalMarkerSize = markerSize
            val rollCmd = if (finalDetected) computeRollValue(finalOffsetX) else 0f

            mainHandler.post {
                imgPreview.setImageBitmap(bitmap)
                if (finalDetected) {
                    val stickPercent = (rollCmd / Stick.MAX_STICK_POSITION_ABS) * 100f
                    tvOffset.text = "Offset: %.2f | Stick: %.1f%% | Size: %.0fpx".format(
                        finalOffsetX, stickPercent, finalMarkerSize)
                    tvStatus.text = if (isCentered) {
                        val elapsed = (System.currentTimeMillis() - centerStartTime) / 1000f
                        "CENTERED (%.1fs / 3.0s) > Auto-land".format(elapsed)
                    } else {
                        "TRACKING | Roll: %.1f%%".format(stickPercent)
                    }
                } else {
                    tvOffset.text = "NO MARKER DETECTED"
                    tvStatus.text = "Searching for marker ID $TARGET_MARKER_ID..."
                }
            }

            bgrMat.release()
            rgbMat.release()
            ids.release()
            corners.forEach { it.release() }

        } catch (e: Exception) {
            Log.e(TAG, "Frame processing error: ${e.message}", e)
        } finally {
            isProcessing.set(false)
        }
    }

    /**
     * PD Controller:
     *   output = P * error + D * (error - prevError)
     *
     * P term: proportional to how far off-center the marker is
     *   - small offset → small correction
     *   - large offset → larger correction (but clamped)
     *
     * D term: proportional to how fast the offset is changing
     *   - marker moving fast → extra push to catch up
     *   - marker slowing down → brakes to prevent overshoot
     */
    private fun computeRollValue(offsetX: Float): Float {
        // Dead zone — ignore tiny offsets
        if (abs(offsetX) < DEAD_ZONE) {
            prevOffsetX = offsetX
            return 0f
        }

        // P term: proportional to offset
        val pTerm = pGain * offsetX

        // D term: rate of change (dampens oscillation)
        val dTerm = dGain * (offsetX - prevOffsetX)

        // Update previous offset for next frame
        prevOffsetX = offsetX

        // Combined PD output
        var rollRatio = pTerm + dTerm

        // Clamp to max stick ratio — this limits how fast the drone can move
        rollRatio = rollRatio.coerceIn(-MAX_STICK_RATIO, MAX_STICK_RATIO)

        // Convert ratio to stick value (660 = max)
        return rollRatio * Stick.MAX_STICK_POSITION_ABS
    }

    private fun sendRollCommand(offsetX: Float) {
        val rollValue = computeRollValue(offsetX).toInt()
        // Right stick horizontal = roll (left/right)
        virtualStickVM.setRightPosition(rollValue, 0)
        // Left stick = throttle + yaw = 0
        virtualStickVM.setLeftPosition(0, 0)
    }

    private fun checkCentered(offsetX: Float) {
        if (abs(offsetX) < DEAD_ZONE * 2) {
            if (!isCentered) {
                isCentered = true
                centerStartTime = System.currentTimeMillis()
            } else {
                val elapsed = System.currentTimeMillis() - centerStartTime
                if (elapsed >= CENTER_HOLD_TIME_MS) {
                    Log.i(TAG, "Centered for 3s - auto-land!")
                    mainHandler.post {
                        ToastUtils.showToast("Marker centered! Auto-landing...")
                        tvStatus.text = "AUTO-LANDING!"
                    }
                    stopTracking()
                    // Disable virtual stick first, then land
                    virtualStickVM.disableVirtualStick(object : CommonCallbacks.CompletionCallback {
                        override fun onSuccess() {
                            basicAircraftControlVM.startLanding(object :
                                CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                                override fun onSuccess(t: EmptyMsg?) {
                                    mainHandler.post { tvStatus.text = "Landing complete" }
                                }
                                override fun onFailure(error: IDJIError) {
                                    mainHandler.post { tvStatus.text = "Landing failed: $error" }
                                }
                            })
                        }
                        override fun onFailure(error: IDJIError) {
                            // Try landing anyway
                            basicAircraftControlVM.startLanding(object :
                                CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                                override fun onSuccess(t: EmptyMsg?) {
                                    mainHandler.post { tvStatus.text = "Landing complete" }
                                }
                                override fun onFailure(error: IDJIError) {
                                    mainHandler.post { tvStatus.text = "Landing failed: $error" }
                                }
                            })
                        }
                    })
                }
            }
        } else {
            isCentered = false
            centerStartTime = 0L
        }
    }

    private fun startTracking() {
        isTracking.set(true)
        isCentered = false
        centerStartTime = 0L
        prevOffsetX = 0f
        btnStartTrack.text = "Stop Tracking"
        tvStatus.text = "TRACKING - show ArUco marker ID $TARGET_MARKER_ID"
        Log.i(TAG, "Tracking started")
    }

    private fun stopTracking() {
        isTracking.set(false)
        isCentered = false
        centerStartTime = 0L
        prevOffsetX = 0f
        virtualStickVM.setRightPosition(0, 0)
        virtualStickVM.setLeftPosition(0, 0)
        mainHandler.post {
            btnStartTrack.text = "Start Tracking"
            tvStatus.text = "Tracking stopped"
        }
        Log.i(TAG, "Tracking stopped")
    }

    private fun updateGainText() {
        tvGain.text = "P-Gain: %.2f".format(pGain)
        tvDGain.text = "D-Gain: %.2f".format(dGain)
    }

    override fun onDestroyView() {
        super.onDestroyView()
        stopTracking()
        try {
            MediaDataCenter.getInstance().cameraStreamManager.removeFrameListener(frameListener)
        } catch (e: Exception) {
            Log.e(TAG, "Error removing listener: ${e.message}")
        }
        processingThread.quitSafely()
    }
}