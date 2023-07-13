package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Robot

@TeleOp
class LimitSwitchesTest() : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry, isTelemetryEnabled = true)

        //disable the elbow and turret so that we can move it to the limit switches for testing
        robot.elbow.isEnabled = false
        robot.turret.isEnabled = false

        robot.telemetry.addLine("Testing Limit Switches")
        robot.telemetry.update()

        schedule(DetermineArmAngles.UpdateTelemetry(robot) {
            robot.telemetry.addData("Left Limit Switch:", robot.limitSwitches.leftLimitSwitchActive)
            robot.telemetry.addData("Right Limit Switch:", robot.limitSwitches.rightLimitSwitchActive)
            robot.telemetry.addData("Elbow Angle:", robot.elbow.currentAngleDegrees)
            robot.telemetry.addData("Turret Angle:", robot.turret.currentAngle)
            robot.telemetry.update()
            })
        }
}