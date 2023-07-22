package org.firstinspires.ftc.teamcode.opmodes.auto.commands

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.commands.drivetrain.TrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot

class MoveToAutoScoringPosition(val robot : Robot, val alliancePosition: AlliancePosition): ConfigurableCommandBase() {
    override fun configure(): CommandBase {
        val currentPose = robot.drivetrain.poseEstimate
        val sb = robot.drivetrain.trajectorySequenceBuilder(currentPose)
        val endPose = when (alliancePosition) {
            AlliancePosition.LEFT -> Pose2d(-13.0, 46.25, 0.0)
            AlliancePosition.CENTER_RIGHT, AlliancePosition.CENTER_LEFT, AlliancePosition.RIGHT -> throw NotImplementedError()
            else -> throw IllegalStateException()
        }
        val trajectory = sb.strafeRight(3.0).forward(endPose.x - currentPose.x).strafeLeft(endPose.y - currentPose.y).build()
        return SequentialCommandGroup(
                TrajectoryCommand(robot.drivetrain, trajectory)

        )




    }
}