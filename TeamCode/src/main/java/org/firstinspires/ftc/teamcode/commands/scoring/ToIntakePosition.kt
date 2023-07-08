package org.firstinspires.ftc.teamcode.commands.scoring

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.Mode

class ToIntakePosition(private val r: Robot): CommandBase() {
    override fun initialize() {
        addRequirements(r.elbow, r.extension, r.claw, r.wrist)
        r.mode = Mode.INTAKE
        r.elbow.targetAngle = -30.0
        r.extension.position = 0.1
        r.claw.openClaw()
        r.wrist.bend = -10.0
        r.wrist.twist =  if (r.fallenCone) 90.0 else 0.0
    }

    override fun isFinished(): Boolean {
        return r.elbow.atTargetPosition
    }

}