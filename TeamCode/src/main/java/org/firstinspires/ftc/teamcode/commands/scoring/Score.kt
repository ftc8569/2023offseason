package org.firstinspires.ftc.teamcode.commands.scoring

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.Mode

class Score(val r: Robot, val angle: Double, val length: Double, val wrist: Double, val aligner: Double): CommandBase() {
    init {
        addRequirements(r.extension, r.elbow, r.wrist)
    }

    override fun initialize() {
        r.mode = Mode.SCORE
        r.extension.position = length
        r.elbow.targetAngle = angle
        r.wrist.bend = wrist
    }

    override fun isFinished() = r.elbow.atTargetPosition

}