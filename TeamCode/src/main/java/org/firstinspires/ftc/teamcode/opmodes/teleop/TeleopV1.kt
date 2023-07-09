package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.arcrobotics.ftclib.command.*
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveFieldCentric
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveMec
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveMecSnap
import org.firstinspires.ftc.teamcode.commands.scoring.HomeScoring
import org.firstinspires.ftc.teamcode.commands.scoring.Score
import org.firstinspires.ftc.teamcode.commands.scoring.ToIntakePosition
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.Mode
import org.firstinspires.ftc.teamcode.utilities.ScoringConfigs.*
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class TeleopV1 : CommandOpMode() {
    override fun initialize() {
        val driver = GamepadEx(gamepad1)
        val gunner = GamepadEx(gamepad2)
        val drivetrain = Drivetrain(hardwareMap)
        val r = Robot(hardwareMap, telemetry) { drivetrain.getYaw() }


        drivetrain.defaultCommand = DriveMec(
            drivetrain,
            { driver.leftY.pow(2) * sign(driver.leftY) },
            { driver.leftX.pow(2) * sign(driver.leftX) },
            { driver.rightX.pow(2) * sign(driver.rightX) },
        )

//        r.drivetrain.defaultCommand = DriveFieldCentric(
//            r.drivetrain,
//            { driver.leftY.pow(2) * sign(driver.leftY) },
//            { driver.leftX.pow(2) * sign(driver.leftX) },
//            { driver.rightX.pow(2) * sign(driver.rightX) },
//            {r.drivetrain.rawExternalHeading},
//        )


//
//        driver.getGamepadButton(GamepadKeys.Button.A)
//            .whenPressed(InstantCommand({ r.drivetrain.resetHeading() }, r.drivetrain))


        // Turret should maintain field relative angle
//        r.turret.defaultCommand = MaintainAngle(r)
//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(
//            DriveMecSnap(
//                r.drivetrain, 0.0,
//                { driver.leftY.pow(2) * sign(driver.leftY) },
//                { driver.leftX.pow(2) * sign(driver.leftX) },
//            )
//        )


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

        gunner.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(InstantCommand({ r.turret.fieldRelativeTargetAngle += 45.0 }, r.turret))

        gunner.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(InstantCommand({ r.turret.fieldRelativeTargetAngle -= 45.0 }, r.turret))

        gunner.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            ConditionalCommand(
                SequentialCommandGroup(
                    InstantCommand(
                        { r.claw.openClaw(); r.mode = Mode.INTAKE; },
                        r.claw
                    ),
                    HomeScoring(r)
                ),
                SequentialCommandGroup(
                    InstantCommand(
                        { r.claw.closeClaw(); r.mode = Mode.SCORE }, r.claw
                    ),
                    WaitCommand(100),
                    HomeScoring(r)
                )

            ) { r.mode == Mode.SCORE }
        )


    }
}