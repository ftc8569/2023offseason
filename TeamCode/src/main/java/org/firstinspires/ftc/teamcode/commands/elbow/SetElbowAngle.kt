package org.firstinspires.ftc.teamcode.commands.elbow

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem

class SetElbowAngle(private val elbow : ElbowSubsystem, private val angle: Double): CommandBase() {
    init {
        addRequirements(elbow)
    }
    override fun initialize() {
        super.initialize()
        elbow.targetAngleDegrees = angle
    }
    override fun isFinished() : Boolean {
        return elbow.isCloseEnoughToTargetAngle()
    }
}