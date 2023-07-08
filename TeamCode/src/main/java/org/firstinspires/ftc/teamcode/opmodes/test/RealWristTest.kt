package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.DiffWrist
import org.firstinspires.ftc.teamcode.utilities.AxonServo

@TeleOp
class RealWristTest : CommandOpMode() {
    override fun initialize() {
        val t = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val wrist = DiffWrist(
            AxonServo(hardwareMap, "leftWrist", 500.0, 2500.0),
            AxonServo(hardwareMap, "rightWrist", 500.0, 2500.0),
            t
        )
        val gp1 = GamepadEx(gamepad1)
        val rightDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
        val leftDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val upDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val downDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)

        rightDpad.whenPressed(InstantCommand({ wrist.bend = 0.0; wrist.twist = 90.0}, wrist))
        leftDpad.whenPressed(InstantCommand({ wrist.bend = 0.0; wrist.twist = -90.0 }, wrist))
        upDpad.whenPressed(InstantCommand({ wrist.bend = 45.0; wrist.twist = 0.0 }, wrist))
        downDpad.whenPressed(InstantCommand({ wrist.bend = -45.0; wrist.twist = 0.0 }, wrist))
    }
}