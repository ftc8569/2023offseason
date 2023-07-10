package org.firstinspires.ftc.teamcode.commands.turret

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem


class SetTurretAngle(val turret: TurretSubsystem, val ang: Double): CommandBase() {
    init {
        addRequirements(turret)
    }

    override fun initialize(){
        turret.targetAngleDegrees = ang
    }

    override fun isFinished() : Boolean {
        return turret.isCloseEnoughToTargetAngle()
    }


}