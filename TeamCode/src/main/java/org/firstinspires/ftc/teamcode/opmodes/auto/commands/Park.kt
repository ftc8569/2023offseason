package org.firstinspires.ftc.teamcode.opmodes.auto.commands

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.apriltags.ConeNumber
import org.firstinspires.ftc.teamcode.commands.drivetrain.TrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot

enum class AlliancePosition {
    LEFT,
    CENTER_RIGHT,
    CENTER_LEFT,
    RIGHT
}
enum class CenterZone {
    FAR, NEAR
}

class Park(val robot : Robot, val alliancePosition: AlliancePosition, val centerZone: CenterZone = CenterZone.NEAR) : ConfigurableCommandBase() {

    override fun configure(): CommandBase {
        // move the robot to the correct parking position from the current position
        // based on the detected signal cone
        val currentPose = robot.drivetrain.poseEstimate
        val sb = robot.drivetrain.trajectorySequenceBuilder(currentPose)

        val centerParkY = when(centerZone) {
            CenterZone.FAR -> -12.0
            CenterZone.NEAR -> -60.0
        }

        val endPose = when(alliancePosition) {
            AlliancePosition.RIGHT -> when (robot.detectedSignalCone) {
                ConeNumber.ONE -> Pose2d(-12.0, -12.0, 0.0)
                ConeNumber.TWO -> Pose2d(-12.0, -36.0, 0.0)
                ConeNumber.THREE -> Pose2d(-12.0, -60.0, 0.0)
                else -> Pose2d(-36.0, -12.0, 0.0) // guess ONE
            }

            AlliancePosition.CENTER_RIGHT -> when (robot.detectedSignalCone) {
                ConeNumber.ONE -> Pose2d(centerParkY, -60.2, 0.0)
                ConeNumber.TWO -> Pose2d(centerParkY, -84.0, 0.0)
                ConeNumber.THREE -> Pose2d(centerParkY, -36.0, 0.0)
                else -> throw IllegalStateException()
            }

            AlliancePosition.CENTER_LEFT -> when (robot.detectedSignalCone) {
                ConeNumber.ONE -> Pose2d(centerParkY, 12.0, 0.0)
                ConeNumber.TWO -> Pose2d(centerParkY, 36.0, 0.0)
                ConeNumber.THREE -> Pose2d(centerParkY, 12.0, 0.0)
                else -> throw IllegalStateException()
            }

            AlliancePosition.LEFT -> when (robot.detectedSignalCone) {
                ConeNumber.ONE -> Pose2d(-12.0, 60.0, 0.0)
                ConeNumber.TWO -> Pose2d(-12.0, 36.0, 0.0)
                ConeNumber.THREE -> Pose2d(-12.0, 12.0, 0.0)
                else -> Pose2d(-12.0, 60.0, 0.0) // guess ONE
            }

            else -> throw IllegalStateException()
        }
        val trajectory = sb.forward(endPose.x - currentPose.x).strafeLeft(endPose.y - currentPose.y).build()
        return TrajectoryCommand(robot.drivetrain, trajectory)
    }
}