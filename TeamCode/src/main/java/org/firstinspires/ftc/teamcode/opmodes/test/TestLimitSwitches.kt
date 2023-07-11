package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import org.firstinspires.ftc.teamcode.subsystems.Robot

class TestLimitSwitches() : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)
        robot.telemetry.addLine("Testing Limit Switches")
        robot.telemetry.update()

        schedule(DetermineArmAngles.UpdateTelemetry(robot) {
            robot.telemetry.addData("Left Limit Switch:", robot.limitSwitches.leftLimitSwitchState)
            robot.telemetry.addData("Right Limit Switch:", robot.limitSwitches.rightLimitSwitchState)
            robot.telemetry.update()
            })
        }
}