package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
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
        var power = 0.0

        waitForStart()
        while(opModeIsActive() && !isStopRequested){
            gp1.readButtons()
            if(gp1.getButton(GamepadKeys.Button.DPAD_UP)) power += 0.1
            if(gp1.getButton(GamepadKeys.Button.DPAD_DOWN)) power -= 0.1

            telemetry.addData("Power: ", power)
            telemetry.addData("encoder: ", analogInput.voltage / 3.3 * 360)
            telemetry.update()
            servo.power = power
        }
    }
}