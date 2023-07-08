package org.firstinspires.ftc.teamcode.opmodes.auto.test

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commands.drivetrain.TrajectoryCommand
import org.firstinspires.ftc.teamcode.subsystems.Robot

@Autonomous
class TestAuto1: LinearOpMode() {
    val r = Robot(hardwareMap, telemetry)

    override fun runOpMode() {
        waitForStart()

//        val driveToStackCommand = TrajectoryCommand(r.drivetrain, )
    }
}