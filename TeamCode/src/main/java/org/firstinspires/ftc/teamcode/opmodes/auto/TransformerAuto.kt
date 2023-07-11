package org.firstinspires.ftc.teamcode.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.apriltags.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.commands.SetElbowTarget
import org.firstinspires.ftc.teamcode.commands.drivetrain.TrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.scoring.HomeScoring
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.FixedSequentialCommandGroup
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import java.lang.Math.PI

@Autonomous
class TransformerAuto: CommandOpMode() {
    override fun initialize() {
        val cameraMonitorViewId = hardwareMap.appContext.resources
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val cam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get<WebcamName>(WebcamName::class.java, "Webcam 1"),
            cameraMonitorViewId
        )
        val aprilTagPipeline = AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506)

        cam.setPipeline(aprilTagPipeline)
        cam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN)
            }

            override fun onError(errorCode: Int) {}
        })
        var sleeveDetection = 1
        while (opModeInInit()) {
            val detections: List<AprilTagDetection> = aprilTagPipeline.latestDetections
            telemetry.addData("Sleeve", sleeveDetection)
            telemetry.update()
            if (detections.isNotEmpty()) {
                sleeveDetection = detections[0].id
                telemetry.addData("Sleeve", sleeveDetection)
                telemetry.update()
            }
        }
        //////////////// END APRILTAG /////////////////////


        val robot = Robot(hardwareMap, telemetry)
        val startPose = Pose2d(12.0, -83.0, PI/2)
        val firstPoint = Vector2d(12.0, -7.0)
        val drive = robot.drivetrain
        drive.poseEstimate = startPose

        val home = HomeScoring(robot)
        val startToFirstPoint = drive.trajectorySequenceBuilder(startPose)
            .lineToConstantHeading(firstPoint)
            .strafeRight(49.0)
            .build()
        val startToConesCommand = TrajectoryCommand(drive, startToFirstPoint)
        val turnTurret = SetTurretAngle(robot.turret, 60.0)
        val extend = InstantCommand({robot.extension.actualPositionExtensionInches = 0.0; robot.wrist.bendAngleDegrees = -10.0}, robot.extension, robot.wrist)

        /////////// START PARKING CONFIGURATION ///////////
        val park1traj = drive.trajectorySequenceBuilder(Pose2d(firstPoint, PI))
            .strafeLeft(40.0)
            .back(10.0)
            .build()

        val park2traj = drive.trajectorySequenceBuilder(Pose2d(firstPoint, PI))
            .strafeLeft(20.0)
            .back(10.0)
            .build()

        val park3traj = drive.trajectorySequenceBuilder(Pose2d(firstPoint, PI))
            .strafeLeft(5.0)
            .back(10.0)
            .build()

        val parkZone1 = TrajectoryCommand(drive, park1traj)
        val parkZone2 = TrajectoryCommand(drive, park2traj)
        val parkZone3 = TrajectoryCommand(drive, park3traj)

        val parkingMap = mutableMapOf<Any, Command>()
        parkingMap[1] = parkZone1
        parkingMap[2] = parkZone2
        parkingMap[3] = parkZone3

        val parkCommand = SelectCommand(parkingMap) {sleeveDetection}
        /////////// END PARKING CONFIGURATION ///////////


        schedule(
            FixedSequentialCommandGroup(
                home,
                SetTurretAngle(robot.turret, 0.0),
                InstantCommand({robot.wrist.bendAngleDegrees = -30.0}, robot.wrist),
                startToConesCommand,
                turnTurret.deadlineWith(WaitCommand(1000)),
                SetElbowTarget(robot, -26.0),
                extend.deadlineWith(WaitCommand(1000)),
                InstantCommand({robot.claw.openClaw()}, robot.claw),
                WaitCommand(1000),
                InstantCommand({robot.claw.closeClaw()}, robot.claw),
                home,
                parkCommand
            )
        )

    }


}