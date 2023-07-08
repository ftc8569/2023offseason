package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

import org.firstinspires.ftc.teamcode.subsystems.Elbow
//@Disabled
@TeleOp
class ElbowTest: LinearOpMode() {
    override fun runOpMode() {
//        val scheduler = CommandScheduler.getInstance()
//        val elbow = Elbow(MotorEx(hardwareMap, "leftElbow"), MotorEx(hardwareMap, "rightElbow"),)
//        val t = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
//        val gp1 = GamepadEx(gamepad1)
//        waitForStart()
//        while(opModeIsActive() && !isStopRequested){
//            gp1.readButtons()
//
//            if(gp1.wasJustPressed(GamepadKeys.Button.Y)){
//                scheduler.schedule(InstantCommand({elbow.targetAngle += 10}, elbow))
//            } else if (gp1.wasJustPressed(GamepadKeys.Button.A)){
//                scheduler.schedule(InstantCommand({elbow.targetAngle -= 10}, elbow))
//            }
//
//            t.addData("Angle", elbow.currentAngle)
//            t.addData("Target Angle", elbow.targetAngle)
//
//            t.update()
//            scheduler.run()
//        }
//        scheduler.reset()
    }
}