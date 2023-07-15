package org.firstinspires.ftc.teamcode.subsystems

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.apriltags.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.apriltags.ConeNumber
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

class SignalConeDetectorSubSystem(val robot: Robot, val isTelemetryEnabled : Boolean = false) : SubsystemBase()  {
    private val cameraMonitorViewId = robot.hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.packageName)
    val webcam: OpenCvCamera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(
        WebcamName::class.java, "rightWebcam"), cameraMonitorViewId)
    val pipeline = AprilTagDetectionPipeline()

    init {
        webcam.setPipeline(pipeline)

        if(isTelemetryEnabled)
            FtcDashboard.getInstance().startCameraStream(webcam, 5.0)
    }

    fun start(resolution : Size) {
        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(resolution.width, resolution.height, OpenCvCameraRotation.UPSIDE_DOWN)
                FtcDashboard.getInstance().startCameraStream(webcam, 60.0)
            }

            override fun onError(errorCode: Int) {
                if(isTelemetryEnabled)
                    robot.telemetry.addData("SignalSleeveSubSystem Error:", errorCode)
            }
        })
    }
    fun getSignalConeDetected() : ConeNumber {
        if(pipeline.latestDetections.size == 0)
            return ConeNumber.NONE
        return when(pipeline.latestDetections.last().id) {
            1 -> ConeNumber.ONE
            2 -> ConeNumber.TWO
            3 -> ConeNumber.THREE
            else -> ConeNumber.NONE
        }
    }
    fun shutdown() {
        webcam.stopStreaming()
        if(isTelemetryEnabled)
            FtcDashboard.getInstance().stopCameraStream()
        webcam.closeCameraDevice()
    }
}