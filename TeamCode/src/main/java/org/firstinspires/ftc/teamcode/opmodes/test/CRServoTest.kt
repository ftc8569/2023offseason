package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PwmControl

@TeleOp
class CRServoTest: LinearOpMode() {
    override fun runOpMode() {
        val servo = hardwareMap.get(CRServoImplEx::class.java, "extension")
        val analogInput = hardwareMap.get(AnalogInput::class.java, "extension")
        servo.pwmRange = PwmControl.PwmRange(500.0,2500.0)
        servo.direction = DcMotorSimple.Direction.REVERSE
        val gp1 = GamepadEx(gamepad1)

        waitForStart()
        while(opModeIsActive() && !isStopRequested){
            gp1.readButtons()
            val power = gp1.leftY
            telemetry.addData("stick: ", power)
            telemetry.addData("encoder: ", analogInput.voltage / 3.3 * 360)
            telemetry.update()
            servo.power = power
        }
    }
}