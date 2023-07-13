package org.firstinspires.ftc.teamcode.commands.scoring

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.PoleAlignerSubsystem

class SetAligner(private val aligner : PoleAlignerSubsystem, private val angle : Double) : CommandBase() {

    init {
        addRequirements(aligner)
    }

    override fun initialize() {
        aligner.angle = angle
    }

    override fun isFinished(): Boolean {
        return aligner.movementShouldBeComplete()
    }

}