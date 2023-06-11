package org.firstinspires.ftc.teamcode.opmodes

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.DriveMec
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import kotlin.math.exp
import kotlin.math.pow
import kotlin.math.sign

@TeleOp(name="TestDrivetrain")
class TestDrivetrain: CommandOpMode() {

    override fun initialize() {
        val drive = Drivetrain(hardwareMap)
        val gp1 = GamepadEx(gamepad1)

        drive.defaultCommand = DriveMec(drive, { gp1.leftY.pow(2) * sign(gp1.leftY) }, {gp1.leftX.pow(2) * sign(gp1.leftX)}, {gp1.rightX.pow(4) * sign(gp1.rightX)})
    }
}