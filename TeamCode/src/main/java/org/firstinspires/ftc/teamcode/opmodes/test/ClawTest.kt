package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Extension
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import org.firstinspires.ftc.teamcode.utilities.AxonServo

@TeleOp
class ClawTest: CommandOpMode() {
    override fun initialize() {
//        val claw = Claw(AxonServo(hardwareMap, "claw", "claw", 500.0, 2500.0), hardwareMap.get(DigitalChannel::class.java, "beamBreak"))
//        val gp1 = GamepadEx(gamepad1)
//        val a = gp1.getGamepadButton(GamepadKeys.Button.A)
//        val b = gp1.getGamepadButton(GamepadKeys.Button.B)
//
//        a.whenPressed(InstantCommand({claw.openClaw()}, claw))
//        b.whenPressed(InstantCommand({claw.closeClaw()}, claw))

    }

}