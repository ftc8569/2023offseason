package org.firstinspires.ftc.teamcode.commands.elbow

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem

class SetElbowAngle(val elbow : ElbowSubsystem, val ang: Double): CommandBase() {
    init {
        addRequirements(elbow)
    }
    override fun initialize() {
        super.initialize()
        elbow.targetAngleDegrees = ang
    }
    override fun isFinished() : Boolean {
        return elbow.isCloseEnoughToTargetAngle()
    }

}