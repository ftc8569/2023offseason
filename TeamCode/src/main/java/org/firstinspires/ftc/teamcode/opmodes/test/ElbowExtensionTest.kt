package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Robot

@Disabled
@TeleOp
class ElbowExtensionTest: LinearOpMode() {
    override fun runOpMode() {
//        val scheduler = CommandScheduler.getInstance()
//        val r = Robot(hardwareMap, telemetry)
//        val gp1 = GamepadEx(gamepad1)
//
//        waitForStart()
//        while(opModeIsActive() && !isStopRequested){
//            scheduler.run()
//            gp1.readButtons()
//
//            if(gp1.wasJustPressed(GamepadKeys.Button.Y)){
//                scheduler.schedule(InstantCommand({r.elbow.targetAngleDegrees = 50.0}, r.elbow))
//            } else if (gp1.wasJustPressed(GamepadKeys.Button.A)){
//                scheduler.schedule(InstantCommand({r.elbow.targetAngleDegrees = 0.0}, r.elbow))
//            }
//
//            if(gp1.wasJustPressed(GamepadKeys.Button.X)){
//                scheduler.schedule(InstantCommand({r.extension.homeMecahnisms()}, r.extension))
//            } else if (gp1.wasJustPressed(GamepadKeys.Button.B)){
//                scheduler.schedule(InstantCommand({r.extension.length = 0.4}, r.extension))
//            }
//        }
//        scheduler.reset()
  }
}