package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.claw.SetClawPosition
import org.firstinspires.ftc.teamcode.commands.commandgroups.*
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveMecanumSnap
import org.firstinspires.ftc.teamcode.commands.general.MonitorRobotTelemetry
import org.firstinspires.ftc.teamcode.commands.turret.ControlTurretAngle
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ArmStatePositionData
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class CRITeleop : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)
        val driver = GamepadEx(gamepad1)
        val gunner = GamepadEx(gamepad2)

        robot.elbow.isTelemetryEnabled = true

        schedule(MoveToTravel(robot))

        // drive with left joystick snapped to 0 degree heading
        robot.drivetrain.defaultCommand = DriveMecanumSnap(
            robot.drivetrain, 0.0,
            { driver.leftY.pow(2) * sign(driver.leftY) },
            { driver.leftX.pow(2) * sign(driver.leftX) },
        )

        // Move the turret to the angle to the right joystick
        val turretController = ControlTurretAngle(robot.turret) {
            Vector2d(-driver.rightY, driver.rightX)
        }
        schedule(turretController)

        // turn on the telemetry monitoring of the robot
        schedule(MonitorRobotTelemetry(robot))

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
        val beamBreakTrigger = Trigger { robot.claw.holdingCone }.whenActive(SetClawPosition(robot.claw, ClawPositions.HOLD_CONE))

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
            IntakeCone(robot, ArmStatePositionData.INTAKE_FRONT),
        )
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
            MoveToTravel(robot)
        )

        // PICKUP Cone
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            PickupCone(robot)
        )
    }

    fun shouldElbowMoveFirst(armState : ArmState) : Boolean {
        return when(armState) {
            ArmState.SCORE_GROUND -> true
            ArmState.SCORE_LOW -> true
            ArmState.SCORE_MEDIUM -> true
            ArmState.SCORE_HIGH -> false
            else -> true
        }
    }

}