package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.arcrobotics.ftclib.command.*
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveMec
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveMecSnap
import org.firstinspires.ftc.teamcode.commands.scoring.HomeScoring
import org.firstinspires.ftc.teamcode.commands.scoring.Score
import org.firstinspires.ftc.teamcode.commands.scoring.ToIntakePosition
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.Mode
import org.firstinspires.ftc.teamcode.utilities.PostAutoPoses.*
import org.firstinspires.ftc.teamcode.utilities.ScoringConfigs.*
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class TeleopV1 : CommandOpMode() {
    override fun initialize() {
        val driver = GamepadEx(gamepad1)
        val gunner = GamepadEx(gamepad2)

        val robot = Robot(hardwareMap, telemetry)
        zeroStaticValuesForSomePurpose()


        robot.drivetrain.defaultCommand = DriveMec(
            robot.drivetrain,
            { driver.leftY.pow(2) * sign(driver.leftY) },
            { driver.leftX.pow(2) * sign(driver.leftX) },
            // TODO test with driver hub since joystick mapping is weird with dash
            { driver.rightX.pow(2) * sign(driver.rightX) },
        )

        driver.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(InstantCommand({ robot.drivetrain.resetHeading() }, robot.drivetrain))

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(
            DriveMecSnap(
                robot.drivetrain, 0.0,
                { driver.leftY.pow(2) * sign(driver.leftY) },
                { driver.leftX.pow(2) * sign(driver.leftX) },
            )
        )

        gunner.getGamepadButton(GamepadKeys.Button.B).whenPressed(
            ToIntakePosition(robot)
        )

        gunner.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(Score(robot, HIGH_ANGLE, HIGH_LENGTH, HIGH_WRIST, HIGH_ALIGNER))
        gunner.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(Score(robot, MED_ANGLE, MED_LENGTH, MED_WRIST, MED_ALIGNER))
        gunner.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(Score(robot, LOW_ANGLE, LOW_LENGTH, LOW_WRIST, LOW_ALIGNER))
        gunner.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(Score(robot, GROUND_ANGLE, GROUND_LENGTH, GROUND_WRIST, GROUND_ALIGNER))

        gunner.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(InstantCommand({ robot.turret.targetAngleDegrees += 45.0 }, robot.turret))

        gunner.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(InstantCommand({ robot.turret.targetAngleDegrees -= 45.0 }, robot.turret))

//        gunner.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//            ConditionalCommand(
//                SequentialCommandGroup(
//                    InstantCommand(
//                        { robot.claw.openClaw(); robot.mode = Mode.INTAKE; },
//                        robot.claw
//                    ),
//                    HomeScoring(robot)
//                ),
//                SequentialCommandGroup(
//                    InstantCommand(
//                        { robot.claw.closeClaw(); robot.mode = Mode.SCORE }, robot.claw
//                    ),
//                    WaitCommand(100),
//                    HomeScoring(robot)
//                )
//
//            ) { robot.mode == Mode.SCORE }
//        )

    }
    private fun zeroStaticValuesForSomePurpose(){
        TURRET_ANGLE = 0.0
        ELBOW_ANGLE = 0.0
        DRIVETRAIN_HEADING = 0.0
    }
}