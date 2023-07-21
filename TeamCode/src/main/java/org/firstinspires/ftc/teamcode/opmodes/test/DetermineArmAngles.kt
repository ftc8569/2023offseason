package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.claw.SetClawPosition
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveMecanumSnap
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class DetermineArmAngles : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)
        val driver = GamepadEx(gamepad1)
        val gunner = GamepadEx(gamepad2)
        var multiplier = 1.0

        // Coordinate System: +x is forward (away from driver station), +y is left, +theta is counter-clockwise
        // (0,0) is the center of the field.  0.0 radians heading is directly away from the driver station along the +x axis
        // this is where we map the robot coordinate system to the field coordinate system
        val startPose = Pose2d(-87.5, -60.0, 0.0)
        robot.drivetrain.poseEstimate = startPose

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
        val xButtonGunner = gunner.getGamepadButton(GamepadKeys.Button.X)
        val yButtonGunner = gunner.getGamepadButton(GamepadKeys.Button.Y)


        robot.telemetry.addLine("Determining Arm Angles")
        robot.telemetry.update()

        robot.drivetrain.defaultCommand = DriveMecanumSnap(
            robot.drivetrain, 0.0,
            { driver.leftY.pow(2) * sign(driver.leftY) },
            { driver.leftX.pow(2) * sign(driver.leftX) },
        )

//        robot.elbow.isTelemetryEnabled = true
//        robot.turret.isTelemetryEnabled = true
//        robot.extension.isTelemetryEnabled = true

        rightBumper.whenPressed(InstantCommand({ multiplier += 1.0;  }))
        leftBumper.whenPressed(InstantCommand({ multiplier -= 1.0;  }))

        // right/left is turret movement
        rightDpad.whenPressed(InstantCommand({ robot.turret.targetAngle += 1.0 * multiplier }, robot.turret))
        leftDpad.whenPressed(InstantCommand({ robot.turret.targetAngle -= 1.0 * multiplier }, robot.turret))

        // triggers are grasp cone
        val triggerThreshold = 0.2
        val driverRightTrigger = Trigger { driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > triggerThreshold }
        val driverLeftTrigger = Trigger { driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > triggerThreshold }
        driverRightTrigger.whenActive(SetClawPosition(robot.claw, ClawPositions.HOLD_CONE))
        driverLeftTrigger.whenActive(SetClawPosition(robot.claw, ClawPositions.OPEN_FOR_INTAKE))

        // up/down is elbow movement
        upDpad.whenPressed(InstantCommand({ robot.elbow.targetAngle += 1.0 * multiplier}, robot.elbow))
        downDpad.whenPressed(InstantCommand({  robot.elbow.targetAngle -= 1.0 * multiplier}, robot.elbow))

        // a/b is wrist movement
        aButton.whenPressed(InstantCommand({ robot.wrist.bendAngleDegrees += 1.0 * multiplier}, robot.wrist))
        bButton.whenPressed(InstantCommand({ robot.wrist.bendAngleDegrees -= 1.0 * multiplier}, robot.wrist))

        // x/y is wrist twist
        xButtonGunner.whenPressed(InstantCommand({ robot.wrist.twistAngleDegrees += 1.0 * multiplier}, robot.wrist))
        yButtonGunner.whenPressed(InstantCommand({ robot.wrist.twistAngleDegrees -= 1.0 * multiplier}, robot.wrist))

        // a/b gunner is pole aligner
        aButtonGunner.whenPressed(InstantCommand({ robot.aligner.angle += 1.0 * multiplier}, robot.aligner))
        bButtonGunner.whenPressed(InstantCommand({ robot.aligner.angle -= 1.0 * multiplier}, robot.aligner))

        // x/y is extension movement
        xButton.whenPressed(InstantCommand({ robot.extension.targetLength -= 0.25 * multiplier }, robot.extension))
        yButton.whenPressed(InstantCommand({ robot.extension.targetLength += 0.25 * multiplier }, robot.extension))

        schedule(UpdateTelemetry(robot) {
            robot.telemetry.addData("X", robot.drivetrain.poseEstimate.x)
            robot.telemetry.addData("Y", robot.drivetrain.poseEstimate.y)
            robot.telemetry.addData("Turret Angle", robot.turret.currentAngle)
            robot.telemetry.addData("Elbow Angle", robot.elbow.currentAngleDegrees)
            robot.telemetry.addData("Wrist Bend Angle", robot.wrist.bendAngleDegrees)
            robot.telemetry.addData("Extension Position", robot.extension.targetLength)
            robot.telemetry.addData("Pole Aligner Angle", robot.aligner.angle)
            robot.telemetry.addData("angleMultiplier", multiplier)
            robot.telemetry.addData("Wrist Twist Angle", robot.wrist.twistAngleDegrees)
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