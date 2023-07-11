package org.firstinspires.ftc.teamcode.commands.turret

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem

class ControlTurretAngle(val turret : TurretSubsystem,  val turrentAngleProvider : () -> Vector2d) : CommandBase() {
    init {
        addRequirements(turret)
    }
    override fun execute() {
        turret.targetAngleDegrees = turrentAngleProvider.invoke().angle()
    }
    override fun isFinished(): Boolean {
        return false
    }


}