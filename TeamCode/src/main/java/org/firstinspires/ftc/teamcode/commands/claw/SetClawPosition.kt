package org.firstinspires.ftc.teamcode.commands.claw

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.Robot


class SetClawPosition(private val claw : ClawSubsystem, private val position: ClawPositions) : CommandBase() {
    init {
        addRequirements(claw)
    }
    override fun execute() {
        claw.position = position
    }
    override fun isFinished() : Boolean {
        return claw.movementShouldBeComplete()
    }
}