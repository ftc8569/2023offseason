package org.firstinspires.ftc.teamcode.commands.extension

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.ExtensionLinkageSubsystem

class SetExtensionLinkage(private val linkage : ExtensionLinkageSubsystem, private val length_in : Double) : CommandBase() {
    init {
        addRequirements(linkage)
    }

    override fun initialize() {
        linkage.targetLength = length_in;
        println("SetExtensionLinkage: distance " + linkage.targetLength)
    }

    override fun isFinished(): Boolean {
        return linkage.movementShouldBeComplete()
    }

}