package org.firstinspires.ftc.teamcode.commands.scoring

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot

class Score(val r: Robot, val angle: Double, val length: Double, val wrist: Double, val aligner: Double): CommandBase() {
    init {
        addRequirements(r.extension, r.elbow, r.wrist)
    }

    override fun initialize() {
        r.extension.targetLength = length
        r.elbow.targetAngle = angle
        r.wrist.bendAngleDegrees = wrist
        r.aligner.angle = r.aligner.convertPositionToAngle(aligner)
    }

    override fun isFinished() = r.elbow.isCloseEnoughToTargetAngle()

}