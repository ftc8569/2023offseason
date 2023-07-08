package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveMec
import org.firstinspires.ftc.teamcode.commands.scoring.Score
import org.firstinspires.ftc.teamcode.commands.scoring.ToIntakePosition
import org.firstinspires.ftc.teamcode.commands.turret.MaintainAngle
import org.firstinspires.ftc.teamcode.commands.turret.TurretLock45
import org.firstinspires.ftc.teamcode.commands.turret.TurretLock90
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.HelperFunctions
import org.firstinspires.ftc.teamcode.utilities.Mode
import org.firstinspires.ftc.teamcode.utilities.ScoringConfigs.*
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class TeleopV1 : CommandOpMode() {
    override fun initialize() {
        val driver = GamepadEx(gamepad1)
        val gunner = GamepadEx(gamepad2)
        val r = Robot(hardwareMap, telemetry)

        r.drivetrain.defaultCommand = DriveMec(
            r.drivetrain,
            { driver.leftY.pow(2) * sign(driver.leftY) },
            { driver.leftX.pow(2) * sign(driver.leftX) },
            { driver.rightX.pow(4) * sign(driver.rightX) },
        )

        // Turret should maintain field relative angle
        r.turret.defaultCommand = MaintainAngle(r)

        Trigger { gunner.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 }.whenActive(
            ConditionalCommand(
                TurretLock45(
                    r.turret,
                    { HelperFunctions.toRadians(r.drivetrain.getYaw()) },
                    { Vector2d(gunner.leftX, gunner.leftY) }),
                TurretLock90(r.turret,
                    { HelperFunctions.toRadians(r.drivetrain.getYaw()) },
                    { Vector2d(gunner.leftX, gunner.leftY) })
            ) { r.mode == Mode.SCORE }
        )

        gunner.getGamepadButton(GamepadKeys.Button.B).whenPressed(
            ToIntakePosition(r)
        )

        gunner.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(Score(r, HIGH_ANGLE, HIGH_LENGTH, HIGH_WRIST, HIGH_ALIGNER))
        gunner.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(Score(r, MED_ANGLE, MED_LENGTH, MED_WRIST, MED_ALIGNER))
        gunner.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(Score(r, LOW_ANGLE, LOW_LENGTH, LOW_WRIST, LOW_ALIGNER))
        gunner.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(Score(r, GROUND_ANGLE, GROUND_LENGTH, GROUND_WRIST, GROUND_ALIGNER))

        gunner.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            ConditionalCommand(
                InstantCommand(
                    { r.claw.openClaw(); r.mode = Mode.INTAKE; r.extension.home() },
                    r.claw,
                    r.extension
                ),
                InstantCommand(
                    { r.claw.closeClaw(); r.mode = Mode.SCORE }, r.claw
                )
            ) { r.mode == Mode.SCORE }
        )

        gunner.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(InstantCommand({ r.fallenCone = !r.fallenCone }))


    }
}