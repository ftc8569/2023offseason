package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SelectCommand
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.commandgroups.*
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveMecanumSnap
import org.firstinspires.ftc.teamcode.commands.turret.ControlTurretAngle
import org.firstinspires.ftc.teamcode.subsystems.ArmAndTurretStateData
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ArmStates
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class TeleopV2 : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)

        val driver = GamepadEx(gamepad1)
        val gunner = GamepadEx(gamepad2)

        // drive with left joystick snapped to 0 degree heading
        robot.drivetrain.defaultCommand = DriveMecanumSnap(
            robot.drivetrain, 0.0,
            { driver.leftY.pow(2) * sign(driver.leftY) },
            { driver.leftX.pow(2) * sign(driver.leftX) },
        )

        // Move the turret to the angle to the right joystick
        schedule(ControlTurretAngle(robot.turret) {
            Vector2d(-driver.rightY, driver.rightX)
        })


        // Score high, medium, or low based on the gamepad buttons
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            ScoreHighJunction(robot)
        )
        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
            ScoreMediumJunction(robot)
        )
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            ScoreLowJunction(robot)
        )
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
            ScoreGroundJunction(robot)
        )
        // Deposit
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            DepositCone(robot)
        )

        // Intake position based on the DPAD
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
            IntakeCone(robot, ArmStates.INTAKE_REAR)
        )
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
            IntakeCone(robot, ArmStates.INTAKE_FRONT)
        )
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
            IntakeCone(robot, ArmStates.INTAKE_LEFT)
        )
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
            IntakeCone(robot, ArmStates.INTAKE_RIGHT)
        )

        // PICKUP Cone
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            PickupCone(robot)
        )
    }

    fun shouldElbowMoveFirst(armState : ArmState) : Boolean {
        return when(armState) {
            ArmState.GROUND -> true
            ArmState.LOW -> true
            ArmState.MED -> true
            ArmState.HIGH -> false
            else -> true
        }
    }

}