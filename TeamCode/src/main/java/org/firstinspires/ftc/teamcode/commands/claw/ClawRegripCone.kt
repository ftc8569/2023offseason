package org.firstinspires.ftc.teamcode.commands.claw

import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.Robot

class ClawRegripCone(robot : Robot) : SequentialCommandGroup() {
    init {
        addCommands(
            SetClawPosition(robot.claw, ClawPositions.RELEASE_CONE_BUT_HOLD_TSE),
            WaitCommand(100),
            SetClawPosition(robot.claw, ClawPositions.HOLD_CONE)
        )
    }
}