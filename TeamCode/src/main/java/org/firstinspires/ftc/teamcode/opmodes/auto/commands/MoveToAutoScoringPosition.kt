package org.firstinspires.ftc.teamcode.opmodes.auto.commands

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ScheduleCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.commands.drivetrain.TrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.abs

class MoveToAutoScoringPosition(val robot : Robot, val alliancePosition: AlliancePosition): ConfigurableCommandBase() {
    override fun configure(): CommandBase {
        val currentPose = robot.drivetrain.poseEstimate
        val sb = robot.drivetrain.trajectorySequenceBuilder(currentPose)
        val endPose = when (alliancePosition) {
            AlliancePosition.LEFT -> Pose2d(-13.0, 46.75, 0.0)
            AlliancePosition.RIGHT -> Pose2d(-12.8, -35.65, 0.0)
            AlliancePosition.CENTER_RIGHT, AlliancePosition.CENTER_LEFT -> throw NotImplementedError()
            else -> throw IllegalStateException()
        }

        val trajectory = when (alliancePosition) {
            AlliancePosition.LEFT ->  sb.strafeRight(3.0).forward(endPose.x - currentPose.x).strafeLeft(endPose.y - currentPose.y).build()
            AlliancePosition.RIGHT -> sb.strafeRight(3.0).forward(endPose.x - currentPose.x).strafeLeft( endPose.y - currentPose.y).build()
            else -> throw IllegalStateException()
        }

        return  TrajectoryCommand(robot.drivetrain, trajectory)

    }
}