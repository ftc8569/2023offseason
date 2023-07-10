package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.abs


class SetElbowTarget(val r: Robot, val ang: Double): CommandBase() {
    init {
        addRequirements(r.turret)
    }

    override fun initialize(){
        r.elbow.targetAngleDegrees = ang
    }

    override fun isFinished() : Boolean {
        return r.elbow.isCloseEnoughToTargetAngle()
    }

}