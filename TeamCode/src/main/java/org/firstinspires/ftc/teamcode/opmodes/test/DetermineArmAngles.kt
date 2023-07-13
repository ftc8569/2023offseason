package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Robot

@TeleOp
class DetermineArmAngles : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)
        val driver = GamepadEx(gamepad1)
        val gunner = GamepadEx(gamepad2)
        var multiplier = 1.0

        val rightDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
        val leftDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val upDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val downDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
        val aButton = driver.getGamepadButton(GamepadKeys.Button.A)
        val bButton = driver.getGamepadButton(GamepadKeys.Button.B)
        val xButton = driver.getGamepadButton(GamepadKeys.Button.X)
        val yButton = driver.getGamepadButton(GamepadKeys.Button.Y)
        val leftBumper = driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
        val rightBumper = driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)

        val aButtonGunner = gunner.getGamepadButton(GamepadKeys.Button.A)
        val bButtonGunner = gunner.getGamepadButton(GamepadKeys.Button.B)


        robot.telemetry.addLine("Determining Arm Angles")
        robot.telemetry.update()

        robot.elbow.isTelemetryEnabled = true
//        robot.turret.isTelemetryEnabled = true
        rightBumper.whenPressed(InstantCommand({ multiplier += 1.0;  }))
        leftBumper.whenPressed(InstantCommand({ multiplier -= 1.0;  }))

        // right/left is turret movement
        rightDpad.whenPressed(InstantCommand({ robot.turret.targetAngle += 1.0 * multiplier }, robot.turret))
        leftDpad.whenPressed(InstantCommand({ robot.turret.targetAngle -= 1.0 * multiplier }, robot.turret))

        // up/down is elbow movement
        upDpad.whenPressed(InstantCommand({ robot.elbow.targetAngle += 1.0 * multiplier}, robot.elbow))
        downDpad.whenPressed(InstantCommand({  robot.elbow.targetAngle -= 1.0 * multiplier}, robot.elbow))

        // a/b is wrist movement
        aButton.whenPressed(InstantCommand({ robot.wrist.bendAngleDegrees += 1.0 * multiplier}, robot.wrist))
        bButton.whenPressed(InstantCommand({ robot.wrist.bendAngleDegrees -= 1.0 * multiplier}, robot.wrist))

        // a/b gunner is pole aligner
        aButtonGunner.whenPressed(InstantCommand({ robot.aligner.angle += 1.0 * multiplier}, robot.aligner))
        bButtonGunner.whenPressed(InstantCommand({ robot.aligner.angle -= 1.0 * multiplier}, robot.aligner))

        // x/y is extension movement
        xButton.whenPressed(InstantCommand({ robot.extension.targetLength -= 0.25 * multiplier }, robot.extension))
        yButton.whenPressed(InstantCommand({ robot.extension.targetLength += 0.25 * multiplier }, robot.extension))

        schedule(UpdateTelemetry(robot) {
            robot.telemetry.addData("Turret Angle", robot.turret.currentAngle)
            robot.telemetry.addData("Elbow Angle", robot.elbow.currentAngleDegrees)
            robot.telemetry.addData("Wrist Bend Angle", robot.wrist.bendAngleDegrees)
            robot.telemetry.addData("Extension Position", robot.extension.targetLength)
            robot.telemetry.addData("Pole Aligner Angle", robot.aligner.angle)
            robot.telemetry.addData("angleMultiplier", multiplier)
            robot.telemetry.update()
        })
    }

    class UpdateTelemetry(val robot : Robot, val printTelemetry : () -> Unit ) : CommandBase() {
        override fun execute() {
            printTelemetry()
        }

        override fun isFinished(): Boolean {
            return false
        }
    }
}