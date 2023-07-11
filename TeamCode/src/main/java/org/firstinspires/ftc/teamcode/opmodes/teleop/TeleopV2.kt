package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.commandgroups.*
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveMec
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveMecSnap
import org.firstinspires.ftc.teamcode.commands.turret.ControlTurretAngle
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class TeleopV2 : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)

        val driver = GamepadEx(gamepad1)
        val gunner = GamepadEx(gamepad2)

        // drive snapped to 0 degrees heading
        robot.drivetrain.defaultCommand = DriveMecSnap(
            robot.drivetrain, 0.0,
            { driver.leftY.pow(2) * sign(driver.leftY) },
            { driver.leftX.pow(2) * sign(driver.leftX) },
        )

        // Move the turret to the angle to the right joystick
        schedule(ControlTurretAngle(robot.turret) {
            Vector2d(-gunner.rightY, gunner.rightX)
        })


//        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//            ScoreHighJunction(robot)
//        )
//
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
//            ScoreMediumJunction(robot)
//        )
//
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//            ScoreLowJunction(robot)
//        )
//
//        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//            DepositCone(robot)
//        )
//
//        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
//            TeleopIntakeCone(robot)
//        )
//
//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//            PickupCone(robot)
//        )
//
//
//







    }
}