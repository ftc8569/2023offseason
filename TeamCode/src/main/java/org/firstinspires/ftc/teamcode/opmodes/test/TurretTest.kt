package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Robot

@TeleOp
class TurretTest: CommandOpMode() {
    override fun initialize() {

        val robot = Robot(hardwareMap, telemetry)

        val driver = GamepadEx(gamepad1)
        val gunner = GamepadEx(gamepad2)

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