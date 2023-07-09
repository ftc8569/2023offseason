package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.subsystems.Turret

@TeleOp
class TurretTest: CommandOpMode() {
    override fun initialize() {
//        val r = Robot(hardwareMap, telemetry)
//        val gp1 = GamepadEx(gamepad1)
//        val rightDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//        val leftDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//        val upDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//        val downDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//
//        rightDpad.whenPressed(InstantCommand({r.turret.targetAngle = 90.0}, r.turret))
//        leftDpad.whenPressed(InstantCommand({r.turret.targetAngle = -90.0}, r.turret))
//        upDpad.whenPressed(InstantCommand({r.turret.targetAngle = 5.0}, r.turret))
//        downDpad.whenPressed(InstantCommand({r.turret.targetAngle = -180.0}, r.turret))
    }

}