package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.geometry.Vector2d
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.TurretLock45
import org.firstinspires.ftc.teamcode.subsystems.Turret

@TeleOp
class TurretLock45Test: CommandOpMode() {
    override fun initialize() {
        val turret = Turret(MotorEx(hardwareMap, "turret"))
        val gp1 = GamepadEx(gamepad1)
        turret.defaultCommand = TurretLock45(turret, {0.0},{ Vector2d(gp1.leftX, gp1.leftY) } )

    }
}