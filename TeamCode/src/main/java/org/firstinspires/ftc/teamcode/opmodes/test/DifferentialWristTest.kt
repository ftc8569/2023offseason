package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Robot

@TeleOp()
class DifferentialWristTest : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)
        val driver = GamepadEx(gamepad1)
        val rightDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
        val leftDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val upDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val downDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)

        robot.wrist.isTelemetryEnabled = true
        robot.telemetry.addLine("Differential Wrist Test Initialized")
        robot.telemetry.update()

        rightDpad.whenPressed(InstantCommand({ robot.wrist.bendAngleDegrees = 0.0; robot.wrist.twistAngleDegrees = 90.0}, robot.wrist))
        leftDpad.whenPressed(InstantCommand({ robot.wrist.bendAngleDegrees = 0.0; robot.wrist.twistAngleDegrees = -90.0 }, robot.wrist))
        upDpad.whenPressed(InstantCommand({ robot.wrist.bendAngleDegrees = 45.0; robot.wrist.twistAngleDegrees = 0.0 }, robot.wrist))
        downDpad.whenPressed(InstantCommand({ robot.wrist.bendAngleDegrees = -45.0; robot.wrist.twistAngleDegrees = 0.0 }, robot.wrist))
    }
}