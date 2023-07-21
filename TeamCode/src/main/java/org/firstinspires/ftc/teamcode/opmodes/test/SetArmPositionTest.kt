package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.commands.arm.SetArmPosition
import org.firstinspires.ftc.teamcode.commands.turret.ControlTurretAngle
import org.firstinspires.ftc.teamcode.subsystems.Robot

class SetArmPositionTest() : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)
        val driver = GamepadEx(gamepad1)
        val gunner = GamepadEx(gamepad2)
        var multiplier = 1.0
        var radius = 400.0 // mm
        var zHeight = 300.0 //mm
        val increment = 10 // mm

        val rightDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
        val leftDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val upDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val downDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)

        val leftBumper = driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
        val rightBumper = driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)

        rightBumper.whenPressed(InstantCommand({ multiplier += 1.0;  }))
        leftBumper.whenPressed(InstantCommand({ multiplier -= 1.0;  }))

        // Move the turret to the angle to the right joystick
        val turretController = ControlTurretAngle(robot) {
            Vector2d(-driver.rightY, driver.rightX)
        }
        turretController.schedule(false)

        // use the dpad to move the arm by changing r and zheight values for the arm
        rightDpad.whenPressed(SequentialCommandGroup(InstantCommand({ radius += increment * multiplier }), SetArmPosition(robot, {radius}, {zHeight})))
        leftDpad.whenPressed(SequentialCommandGroup(InstantCommand({ radius -= increment * multiplier }), SetArmPosition(robot, {radius}, {zHeight})))
        upDpad.whenPressed(SequentialCommandGroup(InstantCommand({ zHeight += increment * multiplier }), SetArmPosition(robot, {radius}, {zHeight})))
        downDpad.whenPressed(SequentialCommandGroup(InstantCommand({ zHeight -= increment * multiplier }), SetArmPosition(robot, {radius}, {zHeight})))

        schedule(DetermineArmAngles.UpdateTelemetry(robot) {
            robot.telemetry.addData("R (mm)", radius)
            robot.telemetry.addData("Z (mm)", zHeight)
            robot.telemetry.addData("Turret Angle", robot.turret.currentAngle)
            robot.telemetry.addData("Elbow Angle", robot.elbow.currentAngleDegrees)
            robot.telemetry.addData("Wrist Bend Angle", robot.wrist.bendAngleDegrees)
            robot.telemetry.addData("Extension Position", robot.extension.targetLength)
            robot.telemetry.addData("Multiplier", multiplier)
        })
    }
}