package org.firstinspires.ftc.teamcode.opmodes.auto.commands

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.commands.drivetrain.TrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot

class MoveToAutoScoringPosition(val robot : Robot, val alliancePosition: AlliancePosition): ConfigurableCommandBase() {
    override fun configure(): CommandBase {
        val currentPose = robot.drivetrain.poseEstimate
        val sb = robot.drivetrain.trajectorySequenceBuilder(currentPose)
        val endPose = when (alliancePosition) {
            AlliancePosition.RIGHT -> Pose2d(-37.0, -64.0, 0.0)
            AlliancePosition.CENTER_RIGHT, AlliancePosition.CENTER_LEFT, AlliancePosition.LEFT -> throw NotImplementedError()
            else -> throw IllegalStateException()
        }
        val trajectory = sb.forward(endPose.x - currentPose.x).strafeLeft(endPose.y - currentPose.y).build()
        return TrajectoryCommand(robot.drivetrain, trajectory)
    }
}