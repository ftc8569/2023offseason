package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PwmControl

@Disabled
@TeleOp
class CRServoTest: LinearOpMode() {
    override fun runOpMode() {
        val servo = hardwareMap.get(CRServoImplEx::class.java, "rightWrist")
        val analogInput = hardwareMap.get(AnalogInput::class.java, "rightWrist")
        servo.pwmRange = PwmControl.PwmRange(500.0,2500.0)
        servo.direction = DcMotorSimple.Direction.REVERSE
        val gp1 = GamepadEx(gamepad1)
        var power = 0.0
        val t = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)

        waitForStart()
        while(opModeIsActive() && !isStopRequested){
            gp1.readButtons()
            if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) power += 0.1
            if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) power -= 0.1

            t.addData("Power: ", power)
            t.addData("encoder: ", analogInput.voltage / 3.3 * 360)
            t.update()
            servo.power = power
        }
    }
}