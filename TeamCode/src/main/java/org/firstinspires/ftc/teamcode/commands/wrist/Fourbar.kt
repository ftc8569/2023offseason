package org.firstinspires.ftc.teamcode.commands.wrist

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.DifferentialWristSubsystem
import org.firstinspires.ftc.teamcode.subsystems.Robot

class Fourbar(private val wrist: DifferentialWristSubsystem, val robot: Robot): CommandBase() {
//    override fun execute(){
//        val elbowAngle = robot.elbow.currentAngle
//        wrist.setTargets(-elbowAngle, -elbowAngle)
//    }
//    override fun isFinished() = false
}