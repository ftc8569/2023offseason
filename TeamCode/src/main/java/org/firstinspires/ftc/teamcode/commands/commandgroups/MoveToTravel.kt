package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
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
        return ParallelCommandGroup(
            SetWristAngles(robot.wrist,ArmStatePositionData.TRAVEL.wrist.bendAngle,ArmStatePositionData.TRAVEL.wrist.twistAngle),
            SetExtensionLinkage(robot.extension, ArmStatePositionData.TRAVEL.extension.length),
            SetAligner(robot.aligner, ArmStatePositionData.TRAVEL.aligner.angle),
            SetElbowAngle(robot.elbow, ArmStatePositionData.TRAVEL.elbow.angle)
        )
    }
}