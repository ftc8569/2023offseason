package org.firstinspires.ftc.teamcode.opmodes.auto.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.Robot

class CloseClawOnBeamBreak(val robot: Robot) : CommandBase() {
    init {
        addRequirements(robot.claw)
    }
    override fun execute() {
        if(robot.claw.holdingCone) {
            robot.claw.position = ClawPositions.HOLD_CONE
        }
    }
    override fun isFinished(): Boolean {
        return robot.claw.position == ClawPositions.HOLD_CONE
    }
}