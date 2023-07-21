package org.firstinspires.ftc.teamcode.opmodes.auto.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.AlliancePosition
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.DepositHighPoleAuto
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.IntakeFromConeStack
import org.firstinspires.ftc.teamcode.subsystems.OpModeType
import org.firstinspires.ftc.teamcode.subsystems.Robot

@TeleOp
class DepositHighPoleAutoTest() : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry, OpModeType.AUTONOMOUS)
        val driver = GamepadEx(gamepad1)
        val alliancePosition = AlliancePosition.RIGHT


        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            DepositHighPoleAuto(robot, alliancePosition)
        )
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            IntakeFromConeStack(robot, alliancePosition, 5),
        )
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
            IntakeFromConeStack(robot, alliancePosition, 4),
        )
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
            IntakeFromConeStack(robot, alliancePosition, 3),
        )
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
            IntakeFromConeStack(robot, alliancePosition, 2),
        )
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
            IntakeFromConeStack(robot, alliancePosition, 1),
        )

        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
            SequentialCommandGroup(
                IntakeFromConeStack(robot, alliancePosition, 5),
                DepositHighPoleAuto(robot, alliancePosition),
                IntakeFromConeStack(robot, alliancePosition, 4),
                DepositHighPoleAuto(robot, alliancePosition),
                IntakeFromConeStack(robot, alliancePosition, 3),
                DepositHighPoleAuto(robot, alliancePosition),
                IntakeFromConeStack(robot, alliancePosition, 2),
                DepositHighPoleAuto(robot, alliancePosition),
                IntakeFromConeStack(robot, alliancePosition, 1),
                DepositHighPoleAuto(robot, alliancePosition),
            )        )

    }
}