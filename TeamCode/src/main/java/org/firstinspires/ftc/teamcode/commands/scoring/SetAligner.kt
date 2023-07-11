package org.firstinspires.ftc.teamcode.commands.scoring

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.PoleAlignerSubsystem

class SetAligner(private val aligner : PoleAlignerSubsystem, private val position : Double) : CommandBase() {

    init {
        addRequirements(aligner)
    }

    val timer = ElapsedTime()

    val duration = 0.15

    override fun initialize() {
        aligner.position = position
        timer.reset()
    }

    override fun isFinished(): Boolean {
        return timer.seconds() > duration
    }

}