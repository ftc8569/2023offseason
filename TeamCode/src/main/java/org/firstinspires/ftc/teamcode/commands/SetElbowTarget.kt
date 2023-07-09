package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot


class SetElbowTarget(val r: Robot, val ang: Double): CommandBase() {
    init {
        addRequirements(r.turret)
    }

    override fun initialize(){
        r.elbow.targetAngle = ang
    }

    override fun isFinished() = (r.elbow.atTargetPosition)


}