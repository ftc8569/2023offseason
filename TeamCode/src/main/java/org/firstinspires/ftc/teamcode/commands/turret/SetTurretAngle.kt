package org.firstinspires.ftc.teamcode.commands.turret

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Turret


class SetTurretAngle(val turret: Turret, val ang: Double): CommandBase() {
    init {
        addRequirements(turret)
    }

    override fun initialize(){
        turret.targetAngle = ang
    }

    override fun isFinished() = (turret.atTarget)


}