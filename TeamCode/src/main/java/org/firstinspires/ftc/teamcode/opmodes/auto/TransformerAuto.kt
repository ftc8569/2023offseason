package org.firstinspires.ftc.teamcode.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.apriltags.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.commands.SetElbowTarget
import org.firstinspires.ftc.teamcode.commands.drivetrain.TrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.scoring.HomeScoring
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.subsystems.AutoDrive
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.FixedSequentialCommandGroup
import org.firstinspires.ftc.teamcode.utilities.HelperFunctions
import org.firstinspires.ftc.teamcode.utilities.PostAutoPoses.*
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import java.lang.Math.PI
import java.time.Instant

@Autonomous
class TransformerAuto: LinearOpMode() {
    val scheduler = CommandScheduler.getInstance()
    override fun runOpMode() {

        //////////////// START APRILTAG /////////////////////
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


        val drive = AutoDrive(hardwareMap)
        val r = Robot(hardwareMap, telemetry) { HelperFunctions.toDegrees(drive.drive.rawExternalHeading) }
        waitForStart()
        val startPose = Pose2d(12.0, -83.0, PI/2)
        val firstPoint = Vector2d(12.0, -7.0)
        drive.drive.poseEstimate = startPose

        val home = HomeScoring(r)
        val startToFirstPoint = drive.drive.trajectorySequenceBuilder(startPose)
            .lineToConstantHeading(firstPoint)
            .strafeRight(49.0)
            .build()
        val startToConesCommand = TrajectoryCommand(drive, startToFirstPoint)
        val turnTurret = SetTurretAngle(r.turret, 60.0)
        val extend = InstantCommand({r.extension.position = 0.0; r.wrist.bend = -10.0}, r.extension, r.wrist)

        /////////// START PARKING CONFIGURATION ///////////
        val park1traj = drive.drive.trajectorySequenceBuilder(Pose2d(firstPoint, PI))
            .strafeLeft(40.0)
            .back(10.0)
            .build()

        val park2traj = drive.drive.trajectorySequenceBuilder(Pose2d(firstPoint, PI))
            .strafeLeft(20.0)
            .back(10.0)
            .build()

        val park3traj = drive.drive.trajectorySequenceBuilder(Pose2d(firstPoint, PI))
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

        val recordSubsystemPositions = InstantCommand(
            {
                DRIVETRAIN_HEADING = HelperFunctions.toDegrees(drive.drive.rawExternalHeading)
                TURRET_ANGLE = r.turret.curAngle
                ELBOW_ANGLE = r.elbow.currentAngle
            },
            r.turret, r.elbow
        )
        scheduler.schedule(
            FixedSequentialCommandGroup(
                home,
                SetTurretAngle(r.turret, 0.0),
                InstantCommand({r.wrist.bend = -30.0}, r.wrist),
                startToConesCommand,
                turnTurret.deadlineWith(WaitCommand(1000)),
                SetElbowTarget(r, -26.0),
                extend.deadlineWith(WaitCommand(1000)),
                InstantCommand({r.claw.openClaw()}, r.claw),
                WaitCommand(1000),
                InstantCommand({r.claw.closeClaw()}, r.claw),
                home,
                parkCommand,
                recordSubsystemPositions
            )
        )

        while(opModeIsActive() && !isStopRequested){
            scheduler.run()
        }
        scheduler.reset()

    }


}