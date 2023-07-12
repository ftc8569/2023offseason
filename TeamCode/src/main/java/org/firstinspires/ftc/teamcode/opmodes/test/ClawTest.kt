package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.claw.SetClawPosition
import org.firstinspires.ftc.teamcode.commands.commandgroups.ScoreGroundJunction
import org.firstinspires.ftc.teamcode.commands.commandgroups.ScoreLowJunction
import org.firstinspires.ftc.teamcode.subsystems.ArmStates
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.Robot

@TeleOp
class ClawTest: CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)

        val driver = GamepadEx(gamepad1)

        val beamBreakTrigger = Trigger { robot.claw.holdingCone }.whenActive(SetClawPosition(robot, ClawPositions.HOLD_CONE))

        robot.claw.isTelemetryEnabled = true
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            SetClawPosition(robot, ClawPositions.OPEN_FOR_INTAKE)
        )
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
            SequentialCommandGroup(SetClawPosition(robot, ClawPositions.RELEASE_CONE_BUT_HOLD_TSE),
                WaitCommand(100),
                SetClawPosition(robot, ClawPositions.HOLD_CONE))
        )
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            SetClawPosition(robot, ClawPositions.HOLD_CONE)
        )

    }

}