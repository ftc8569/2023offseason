package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Turret

@Disabled
@TeleOp
class TurretTest: CommandOpMode() {
    override fun initialize() {
        val turret = Turret(MotorEx(hardwareMap, "turret"))
        val gp1 = GamepadEx(gamepad1)
        val rightDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
        val leftDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val upDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val downDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)

        rightDpad.whenPressed(InstantCommand({turret.targetAngle = 90.0}, turret))
        leftDpad.whenPressed(InstantCommand({turret.targetAngle = -90.0}, turret))
        upDpad.whenPressed(InstantCommand({turret.targetAngle = 5.0}, turret))
        downDpad.whenPressed(InstantCommand({turret.targetAngle = -180.0}, turret))
    }

}