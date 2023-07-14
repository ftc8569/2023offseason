package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.Cons.EASING
import org.firstinspires.ftc.teamcode.commands.claw.ClawRegripCone
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ArmStatePositionData
import org.firstinspires.ftc.teamcode.subsystems.Robot

class MoveToTravel(val robot : Robot) : ConfigurableCommandBase()  {
    override fun initialize() {
        super.initialize()
        robot.armState = ArmState.TRAVEL
    }

    override fun configure(): CommandBase {
        return SequentialCommandGroup(
            SetWristAngles(robot.wrist,ArmStatePositionData.TRAVEL.wrist.bendAngle,ArmStatePositionData.TRAVEL.wrist.twistAngle),
            SetExtensionLinkage(robot.extension, ArmStatePositionData.TRAVEL.extension.length, EASING),
            SetAligner(robot.aligner, ArmStatePositionData.TRAVEL.aligner.angle),
            SetElbowAngle(robot.elbow, ArmStatePositionData.TRAVEL.elbow.angle),
            ConditionalCommand(ClawRegripCone(robot), InstantCommand({ println("MoveToTravel - Skipped Regrip")})) { robot.claw.holdingCone },

        )
    }
}