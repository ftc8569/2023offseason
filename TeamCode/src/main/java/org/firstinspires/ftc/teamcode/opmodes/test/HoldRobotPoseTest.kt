package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.commandgroups.MoveToTravel
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.HoldRobotPose
import org.firstinspires.ftc.teamcode.subsystems.Robot

@TeleOp
class HoldRobotPoseTest() : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)
        MoveToTravel(robot).schedule()
        HoldRobotPose(robot, robot.drivetrain.poseEstimate).schedule()
    }
}