package org.firstinspires.ftc.teamcode.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImpl

@TeleOp
class AlignerTestBasic : LinearOpMode() {
    override fun runOpMode() {


        var servo = hardwareMap.get(Servo::class.java, "aligner")


        waitForStart()
        var servo_pos = 0.5;

        while (opModeIsActive()) {
            if (gamepad1.cross) {
                servo_pos = 0.25
            }
            if (gamepad1.circle) {
                servo_pos = 0.75
            }
            servo.position = servo_pos
        }

    }
}