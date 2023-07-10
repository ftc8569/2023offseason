package org.firstinspires.ftc.teamcode.subsystems.vision

import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.YellowPolePipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

class StereoCams(val robot: Robot) : SubsystemBase() {
    private val cameraMonitorViewId = robot.hardwareMap.appContext.resources.getIdentifier(
        "cameraMonitorViewId",
        "id",
        robot.hardwareMap.appContext.packageName
    )
    private val cameraIds: IntArray = OpenCvCameraFactory.getInstance()
        .splitLayoutForMultipleViewports(
            cameraMonitorViewId,
            2,
            OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY
        )
    private val leftWebcam: OpenCvWebcam = OpenCvCameraFactory.getInstance()
        .createWebcam(robot.hardwareMap.get(WebcamName::class.java, "leftWebcam"), cameraIds[0])
    private val rightWebcam: OpenCvWebcam = OpenCvCameraFactory.getInstance()
        .createWebcam(robot.hardwareMap.get(WebcamName::class.java, "rightWebcam"), cameraIds[1])

    private val leftCameraX = -9
    private val rightCameraX = 9
    var leftSlope = 0.0
    var rightSlope = 0.0
    var xIntersection = 0.0
    var zIntersection = 0.0
    init {
        leftWebcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                leftWebcam.setPipeline(YellowPolePipeline(leftSlope))
                leftWebcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT)
            }

            override fun onError(errorCode: Int) {
            }
        })
        rightWebcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                leftWebcam.setPipeline(YellowPolePipeline(rightSlope))
                leftWebcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT)
            }
            override fun onError(errorCode: Int) {
            }
        })
    }

    override fun periodic(){
        // Calculate the intersection point
        xIntersection =
            (leftCameraX * leftSlope - rightCameraX * rightSlope) / (leftSlope - rightSlope)
        zIntersection = (xIntersection - leftCameraX) * leftSlope

        robot.telemetry.addData("X intersection: ", xIntersection)
        robot.telemetry.addData("Z intersection: ", zIntersection)
        robot.telemetry.update()
    }

    init {
        register()
    }
}