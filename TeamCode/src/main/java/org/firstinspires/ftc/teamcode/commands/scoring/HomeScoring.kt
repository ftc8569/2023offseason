package org.firstinspires.ftc.teamcode.commands.scoring

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.ScoringConfigs.ALIGNER_HOME

class HomeScoring(val r: Robot): CommandBase() {
    init {
        addRequirements(r.elbow, r.extension)
    }

    override fun initialize(){
        r.elbow.targetAngle = 60.0
        r.extension.position = 1.0
        r.aligner.position = ALIGNER_HOME
    }

    override fun isFinished() = r.elbow.atTargetPosition
}