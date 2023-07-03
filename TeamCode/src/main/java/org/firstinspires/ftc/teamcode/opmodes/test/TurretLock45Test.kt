package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.geometry.Vector2d
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.TurretLock45
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.subsystems.Turret

@TeleOp
class TurretLock45Test: CommandOpMode() {
    override fun initialize() {
        val r = Robot(hardwareMap, telemetry)
        val gp1 = GamepadEx(gamepad1)
        r.turret.defaultCommand = TurretLock45(r.turret, {0.0},{ Vector2d(gp1.leftX, gp1.leftY) } )
        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
            ParallelCommandGroup(
                InstantCommand({r.extension.length = 0.4}, r.extension),
                InstantCommand({r.elbow.targetAngle = 50.0}, r.elbow)
            )
        )
        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            ParallelCommandGroup(
                InstantCommand({r.extension.home()}, r.extension),
                InstantCommand({r.elbow.targetAngle = .0}, r.elbow)
            )
        )

    }
}