package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
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

        // Intake position based on the DPAD
        val beamBreakTrigger = Trigger { robot.claw.holdingCone }.whenActive(
            SetClawPosition(
                robot.claw,
                ClawPositions.HOLD_CONE
            )
        )

        // REGULAR SCORING
        Trigger { driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 }.whenActive(
            ScoreHighJunction(robot)
        ).whenInactive(DepositCone(robot))

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenActive(ScoreMediumJunction(robot)).whenInactive(DepositCone(robot))

        driver.getGamepadButton(GamepadKeys.Button.Y).whenHeld(ScoreLowJunction(robot))
            .whenInactive(DepositCone(robot))

        driver.getGamepadButton(GamepadKeys.Button.B).whenHeld(ScoreGroundJunction(robot))
            .whenInactive(DepositCone(robot))

        // CAPPING
        val rightTrigger = Trigger { driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 }

        Trigger { driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 }
            .and(rightTrigger)
            .whenActive(
            ScoreHighJunction(robot)
        ).whenInactive(DepositTSEUnderCone(robot))

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .and(rightTrigger)
            .whenActive(ScoreMediumJunction(robot)).whenInactive(DepositCone(robot))
            .whenInactive(DepositTSEUnderCone(robot))

        driver.getGamepadButton(GamepadKeys.Button.Y).whenHeld(ScoreLowJunction(robot))
            .and(rightTrigger)
            .whenInactive(DepositTSEUnderCone(robot))

        driver.getGamepadButton(GamepadKeys.Button.B).whenHeld(ScoreGroundJunction(robot))
            .and(rightTrigger)
            .whenInactive(DepositTSEUnderCone(robot))



        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenHeld(IntakeCone(robot, ArmStatePositionData.INTAKE_FRONT))

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .and(beamBreakTrigger)
            .whenActive(SequentialCommandGroup(WaitCommand(300), MoveToTravel(robot)))

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
            IntakeCone(robot, ArmStatePositionData.INTAKE_FRONT),
        )
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
            MoveToTravel(robot)
        )

        fun shouldElbowMoveFirst(armState: ArmState): Boolean {
            return when (armState) {
                ArmState.SCORE_GROUND -> true
                ArmState.SCORE_LOW -> true
                ArmState.SCORE_MEDIUM -> true
                ArmState.SCORE_HIGH -> false
                else -> true
            }
        }
    }
}