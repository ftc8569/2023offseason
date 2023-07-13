package org.firstinspires.ftc.teamcode.commands.elbow

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem

class SetElbowAngle(private val elbow : ElbowSubsystem, private val angle: Double): CommandBase() {
    init {
        addRequirements(elbow)
    }
    val timer = ElapsedTime()
    val timeOut = 1.0 // seconds
    override fun initialize() {
        super.initialize()
        elbow.targetAngleDegrees = angle
        timer.reset()
    }
    override fun isFinished() : Boolean {
        return elbow.isCloseEnoughToTargetAngle() || timer.seconds() > timeOut
    }
}