package org.firstinspires.ftc.teamcode.commands.scoring

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.Mode

class ToIntakePosition(private val r: Robot): CommandBase() {
    override fun initialize() {
        addRequirements(r.elbow, r.extension, r.claw, r.wrist)
        r.mode = Mode.INTAKE
        r.elbow.targetAngleDegrees = -30.0
        r.extension.actualPositionExtensionInches = 0.1
        r.claw.openClaw()
        r.wrist.bendAngleDegrees = -10.0
        r.wrist.twistAngleDegrees =  if (r.fallenCone) 90.0 else 0.0
    }

    override fun isFinished(): Boolean {
        return r.elbow.isCloseEnoughToTargetAngle()
    }

}