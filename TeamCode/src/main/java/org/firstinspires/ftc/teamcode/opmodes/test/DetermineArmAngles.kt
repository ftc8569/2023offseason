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
        var angleMultiplier = 45.0
        var lengthMultiplier = 1.0

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

        robot.telemetry.addLine("Determining Arm Angles")
        robot.telemetry.update()

        rightBumper.whenPressed(InstantCommand({ angleMultiplier += 1.0; lengthMultiplier += 0.1 }))
        leftBumper.whenPressed(InstantCommand({ angleMultiplier -= 1.0; lengthMultiplier -= 0.1 }))

        // right/left is turret movement
        robot.turret.isTelemetryEnabled = true
        rightDpad.whenPressed(InstantCommand({ robot.turret.targetAngleDegrees += 1.0 * angleMultiplier }, robot.turret))
        leftDpad.whenPressed(InstantCommand({ robot.turret.targetAngleDegrees -= 1.0 * angleMultiplier }, robot.turret))

        // up/down is elbow movement
        upDpad.whenPressed(InstantCommand({ robot.elbow.targetAngleDegrees += 1.0 * angleMultiplier}, robot.elbow))
        downDpad.whenPressed(InstantCommand({  robot.elbow.targetAngleDegrees -= 1.0 * angleMultiplier}, robot.elbow))

        // a/b is wrist movement
        aButton.whenPressed(InstantCommand({ robot.wrist.bendAngleDegrees += 1.0 * angleMultiplier}, robot.wrist))
        bButton.whenPressed(InstantCommand({ robot.wrist.bendAngleDegrees -= 1.0 * angleMultiplier}, robot.wrist))

        // x/y is extension movement
        xButton.whenPressed(InstantCommand({ robot.extension.actualPositionExtensionInches -= 0.25 * lengthMultiplier }, robot.extension))
        yButton.whenPressed(InstantCommand({ robot.extension.actualPositionExtensionInches += 0.25 * lengthMultiplier }, robot.extension))

        schedule(UpdateTelemetry(robot) {
            robot.telemetry.addData("Turret Angle", robot.turret.currentAngleDegrees)
            robot.telemetry.addData("Elbow Angle", robot.elbow.currentAngleDegrees)
            robot.telemetry.addData("Wrist Bend Angle", robot.wrist.bendAngleDegrees)
            robot.telemetry.addData("Extension Position", robot.extension.actualPositionExtensionInches)
            robot.telemetry.addData("angleMultiplier", angleMultiplier)
            robot.telemetry.addData("lengthMultiplier", lengthMultiplier)
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