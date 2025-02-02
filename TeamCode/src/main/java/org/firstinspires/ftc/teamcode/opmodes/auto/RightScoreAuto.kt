package org.firstinspires.ftc.teamcode.opmodes.auto




import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.drivetrain.TrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.scoring.HomeScoring
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.FixedSequentialCommandGroup
import org.firstinspires.ftc.teamcode.utilities.ScoringConfigs.ALIGNER_SCORE
import java.lang.Math.PI

@Autonomous
@Disabled

class RightScoreAuto: CommandOpMode() {

    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)
        val startPose = Pose2d(12.0, -83.0, PI/2)
        val firstPoint = Vector2d(12.0, -35.0)
        robot.drivetrain.poseEstimate = startPose

        val home = HomeScoring(robot)
        val startToFirstPoint = robot.drivetrain.trajectorySequenceBuilder(startPose)
            .lineToConstantHeading(firstPoint)
            .strafeRight(5.0)
            .build()
        val startToConesCommand = TrajectoryCommand(robot.drivetrain, startToFirstPoint)
        val turnTurret = SetTurretAngle(robot.turret, 90.0)
        val extend = InstantCommand({robot.extension.targetLength = 0.0; robot.wrist.bendAngleDegrees = -10.0}, robot.extension, robot.wrist)
        schedule(FixedSequentialCommandGroup(
            home,
            SetTurretAngle(robot.turret, 0.0),
            InstantCommand({robot.claw.position = ClawPositions.OPEN_FOR_INTAKE}, robot.claw),
            InstantCommand({robot.wrist.bendAngleDegrees = -30.0}, robot.wrist),
            startToConesCommand,
            turnTurret.deadlineWith(WaitCommand(1000)),
            SetElbowAngle(robot.elbow, -15.0),
            extend.deadlineWith(WaitCommand(1000)),
            WaitCommand(1000),
            InstantCommand({robot.claw.position = ClawPositions.HOLD_CONE}, robot.claw),
            WaitCommand(1000),
            SetElbowAngle(robot.elbow, 60.0),
            SetTurretAngle(robot.turret, -45.0),
            InstantCommand({robot.aligner.angle = robot.aligner.convertPositionToAngle(ALIGNER_SCORE) }, robot.aligner),
            InstantCommand({robot.claw.position = ClawPositions.OPEN_FOR_INTAKE}, robot.claw),
            home
        ))
    }


}