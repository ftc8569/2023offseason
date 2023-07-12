package org.firstinspires.ftc.teamcode.commands.general

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot

class MonitorRobotTelemetry(private val robot : Robot) : CommandBase() {
    override fun execute() {
        robot.telemetry.addData("Arm State", robot.armState)
        robot.telemetry.addData("Turret Angle", robot.turret.targetAngleDegrees)
        robot.telemetry.addData("Elbow Angle", robot.elbow.targetAngleDegrees)
        robot.telemetry.addData("Extension Length", robot.extension.targetLength)
        robot.telemetry.addData("Aligner Angle", robot.aligner.angle)
        robot.telemetry.addData("Wrist Bend Angle", robot.wrist.bendAngleDegrees)
        robot.telemetry.addData("Wrist Twist Angle", robot.wrist.twistAngleDegrees)
        robot.telemetry.addData("Claw Position", robot.claw.position)
        robot.telemetry.update()
    }

    override fun isFinished(): Boolean {
        return false
    }
}