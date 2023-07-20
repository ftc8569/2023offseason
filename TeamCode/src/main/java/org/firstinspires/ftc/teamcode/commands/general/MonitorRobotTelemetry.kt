package org.firstinspires.ftc.teamcode.commands.general

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandScheduler
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.subsystems.Robot

class MonitorRobotTelemetry(private val robot : Robot) : CommandBase() {
    override fun execute() {
        val scheduler = CommandScheduler.getInstance()
//        robot.telemetry.addData("require(Turret)", scheduler.requiring(robot.turret))
//        robot.telemetry.addData("require(Elbow)", scheduler.requiring(robot.elbow))
//        robot.telemetry.addData("require(Extension)", scheduler.requiring(robot.extension))
//        robot.telemetry.addData("require(Wrist)", scheduler.requiring(robot.wrist))
//        robot.telemetry.addData("require(Claw)", scheduler.requiring(robot.claw))

        val battery: VoltageSensor = robot.hardwareMap.voltageSensor["Control Hub"]
        val voltage: Double = battery.voltage

        robot.telemetry.addData("Battery Volage", voltage)
        robot.telemetry.addData("Arm State", robot.armState)
        robot.telemetry.addData("Turret Angle", robot.turret.targetAngle)
        robot.telemetry.addData("Elbow Angle", robot.elbow.targetAngle)
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