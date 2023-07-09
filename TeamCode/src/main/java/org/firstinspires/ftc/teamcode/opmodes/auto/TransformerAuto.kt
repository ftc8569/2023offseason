package org.firstinspires.ftc.teamcode.opmodes.auto


import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commands.SetElbowTarget
import org.firstinspires.ftc.teamcode.commands.drivetrain.TrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.scoring.HomeScoring
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.subsystems.AutoDrive
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.FixedSequentialCommandGroup
import org.firstinspires.ftc.teamcode.utilities.HelperFunctions
import java.lang.Math.PI

@Autonomous
class TransformerAuto: LinearOpMode() {
    val scheduler = CommandScheduler.getInstance()
    override fun runOpMode() {
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
                home
            )
        )


        while(opModeIsActive() && !isStopRequested){
            scheduler.run()
        }
        scheduler.reset()

    }


}