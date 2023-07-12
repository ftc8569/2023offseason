package org.firstinspires.ftc.teamcode.commands.claw

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.Robot


class SetClawPosition(private val robot : Robot, private val position: ClawPositions) : CommandBase() {
    init {
        addRequirements(robot.claw)
    }
    override fun initialize() {
        super.initialize()
        robot.claw.position = position
    }
    override fun isFinished() : Boolean {
        return true
    }
}