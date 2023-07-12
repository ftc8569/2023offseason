package org.firstinspires.ftc.teamcode.commands.scoring

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.Robot

class ToIntakePosition(private val r: Robot): CommandBase() {
    override fun initialize() {
        addRequirements(r.elbow, r.extension, r.claw, r.wrist)
        r.elbow.targetAngleDegrees = -30.0
        r.extension.extensionLength = 0.1
        r.claw.position = ClawPositions.OPEN_FOR_INTAKE
        r.wrist.bendAngleDegrees = -10.0
//        r.wrist.twistAngleDegrees =  if (r.fallenCone) 90.0 else 0.0
    }

    override fun isFinished(): Boolean {
        return r.elbow.isCloseEnoughToTargetAngle()
    }

}