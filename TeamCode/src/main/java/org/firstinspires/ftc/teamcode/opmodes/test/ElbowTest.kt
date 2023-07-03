package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Elbow

@TeleOp
class ElbowTest: LinearOpMode() {
    override fun runOpMode() {
//        val scheduler = CommandScheduler.getInstance()
//        val elbow = Elbow(MotorEx(hardwareMap, "leftElbow"), MotorEx(hardwareMap, "rightElbow"))
//        val t = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
//        val gp1 = GamepadEx(gamepad1)
//        var power = 0.0
//        waitForStart()
//        while(opModeIsActive() && !isStopRequested){
//            gp1.readButtons()
//
////            if(gp1.wasJustPressed(GamepadKeys.Button.Y)){
////                elbow.targetAngle = 45.0
////            } else if (gp1.wasJustPressed(GamepadKeys.Button.A)){
////                elbow.targetAngle = 0.0
////            }
//            if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) power += 0.1
//            if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) power -= 0.1
//            elbow.motor.set(power)
//
//            t.addData("Angle", elbow.currentAngle)
//            t.addData("Target Angle", elbow.targetAngle)
//            t.addData("Power", power)
//            t.update()
//            scheduler.run()
//        }
//        scheduler.reset()
    }
}