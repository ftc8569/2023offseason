package org.firstinspires.ftc.teamcode.commands.wrist

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.DifferentialWristSubsystem

class SetWristAngles(val wrist : DifferentialWristSubsystem, val bendAngleDegrees : Double, val twistAngleDegrees: Double) : CommandBase() {
    init {
        addRequirements(wrist)
    }
    val timer = ElapsedTime()
    val estiamtedTimeToFinish = 0.15 // seconds

    override fun initialize() {
        wrist.bendAngleDegrees = bendAngleDegrees
        wrist.twistAngleDegrees = twistAngleDegrees
        timer.reset()
    }

    override fun isFinished(): Boolean {
        return timer.seconds() > estiamtedTimeToFinish
    }
}