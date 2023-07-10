package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.ServoImpl
import org.firstinspires.ftc.teamcode.subsystems.Robot

@Config
@TeleOp
class AlignerTest : CommandOpMode() {

    var servo_pos = 0.5;

    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)

        val gp1 = GamepadEx(gamepad1)
        val a = gp1.getGamepadButton(GamepadKeys.Button.A)
        val b = gp1.getGamepadButton(GamepadKeys.Button.B)
        robot.aligner.servo.position = 0.5

        a.whenPressed(InstantCommand({
            servo_pos += 0.05
            robot.aligner.servo.position = servo_pos
            robot.t.addData("Servo Position", servo_pos)
        }))
        b.whenPressed(InstantCommand({
            servo_pos -= 0.05
            robot.aligner.servo.position = servo_pos
            robot.t.addData("Servo Position", servo_pos)
        }))

        register(robot.aligner)


    }

}

