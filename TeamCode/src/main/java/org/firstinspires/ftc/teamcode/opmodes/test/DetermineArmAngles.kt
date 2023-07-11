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

        val rightDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
        val leftDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val upDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val downDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
        val aButton = driver.getGamepadButton(GamepadKeys.Button.A)
        val bButton = driver.getGamepadButton(GamepadKeys.Button.B)
        val xButton = driver.getGamepadButton(GamepadKeys.Button.X)
        val yButton = driver.getGamepadButton(GamepadKeys.Button.Y)

        robot.telemetry.addLine("Determining Arm Angles")
        robot.telemetry.update()

        // right/left is turret movement
        rightDpad.whenPressed(InstantCommand({ robot.turret.targetAngleDegrees += 1.0 }, robot.turret))
        leftDpad.whenPressed(InstantCommand({ robot.turret.targetAngleDegrees -= 1.0 }, robot.turret))

        // up/down is elbow movement
        upDpad.whenPressed(InstantCommand({ robot.elbow.targetAngleDegrees += 1.0 }, robot.elbow))
        downDpad.whenPressed(InstantCommand({  robot.elbow.targetAngleDegrees -= 1.0 }, robot.elbow))

        // a/b is wrist movement
        aButton.whenPressed(InstantCommand({ robot.wrist.bendAngleDegrees += 1.0 }, robot.wrist))
        bButton.whenPressed(InstantCommand({ robot.wrist.bendAngleDegrees -= 1.0 }, robot.wrist))

        // x/y is extension movement
        robot.extension.isTelemetryEnabled = true
        xButton.whenPressed(InstantCommand({ robot.extension.actualPositionExtensionInches -= 0.25 }, robot.extension))
        yButton.whenPressed(InstantCommand({ robot.extension.actualPositionExtensionInches += 0.25 }, robot.extension))

        schedule(UpdateTelemetry(robot) {
            robot.telemetry.addData("Turret Angle", robot.turret.currentAngleDegrees)
            robot.telemetry.addData("Elbow Angle", robot.elbow.currentAngleDegrees)
            robot.telemetry.addData("Wrist Bend Angle", robot.wrist.bendAngleDegrees)
            robot.telemetry.addData("Extension Position", robot.extension.actualPositionExtensionInches)
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