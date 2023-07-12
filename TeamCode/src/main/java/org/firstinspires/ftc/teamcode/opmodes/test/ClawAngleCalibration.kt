package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.claw.SetClawPosition
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.Robot

@TeleOp
class ClawAngleCalibration : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)

        val driver = GamepadEx(gamepad1)

        robot.claw.isTelemetryEnabled = true
        robot.claw.position = ClawPositions.MANUAL

        var increment = 1.0
        robot.claw.servo.angle = 0.0

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            InstantCommand({ robot.claw.servo.angle -= increment  }, robot.claw)
        )
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
            InstantCommand({ robot.claw.servo.angle = 0.0  }, robot.claw)
        )
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            InstantCommand({ robot.claw.servo.angle += increment  }, robot.claw)
        )

    }
}