package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Robot

//@Disabled
@TeleOp
class ElbowTest: CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)
        val driver = GamepadEx(gamepad1)
        val upDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val downDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
        val bButton = driver.getGamepadButton(GamepadKeys.Button.B)

        robot.elbow.isTelemetryEnabled = true
        robot.telemetry.addLine(" Test Initialized")
        robot.telemetry.update()

        upDpad.whenPressed(InstantCommand({ robot.elbow.targetAngle += 1.0 }, robot.elbow))
        downDpad.whenPressed(InstantCommand({ robot.elbow.targetAngle -= 1.0 }, robot.elbow))
        bButton.whenPressed(InstantCommand({ robot.elbow.targetAngle = 0.0 }, robot.elbow))
    }
}