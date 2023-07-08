package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Robot

@TeleOp
class ScoringHeights: LinearOpMode(){
    var scheduler: CommandScheduler = CommandScheduler.getInstance()
    var angle = 0.0
    var distance = 0.1
    var bend = 0.0

    override fun runOpMode() {
        val gp1 = GamepadEx(gamepad1)
        val robot = Robot(hardwareMap, telemetry)
        waitForStart()
        while(opModeIsActive() && !isStopRequested){
            scheduler.run()
            gp1.readButtons()

            if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                angle += 5.0
            } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                angle -= 5.0
            }

            if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                distance += 0.05
            } else if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                distance -= 0.05
            }

            if(gp1.wasJustPressed(GamepadKeys.Button.Y)){
                bend += 5
            } else if(gp1.wasJustPressed(GamepadKeys.Button.A)){
                bend -= 5
            }

            robot.elbow.targetAngle = angle
            robot.extension.position = distance
            robot.wrist.bend = bend

            robot.t.addData("Angle", angle)
            robot.t.addData("Length", distance)
            robot.t.addData("Bend", bend)
            robot.t.update()

        }
        scheduler.reset()

    }


}