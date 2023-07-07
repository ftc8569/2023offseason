package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo

@TeleOp
class NormalWristServoTest: LinearOpMode() {
    override fun runOpMode() {
        val leftServo = AxonCRServo(hardwareMap, "leftWrist", "leftWrist", 500.0, 2500.0)
        val rightServo = AxonCRServo(hardwareMap, "rightWrist", "rightWrist", 500.0, 2500.0)
        val gp1 = GamepadEx(gamepad1)
        val t = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        leftServo.servo.direction = DcMotorSimple.Direction.REVERSE
        leftServo.analogReversed = true

        var power = 0.0
        waitForStart()
        while(opModeIsActive() && !isStopRequested){
            gp1.readButtons()
            power = gp1.leftY
            leftServo.setPower(power)
            rightServo.setPower(power)
            leftServo.update()
            rightServo.update()

            t.addData("Right Velocity", rightServo.velocity.get())
            t.addData("Left Velocity", leftServo.velocity.get())
            t.addData("Power", power)
            t.update()
        }
    }
}