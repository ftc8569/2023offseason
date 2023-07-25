package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.Cons
import org.firstinspires.ftc.teamcode.commands.claw.ClawRegripCone
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ArmStatePositionData
import org.firstinspires.ftc.teamcode.subsystems.Robot

class MoveToConeRighting(val robot : Robot) : ConfigurableCommandBase() {
    override fun initialize() {
        super.initialize()
        robot.armState = ArmState.CONE_RIGHTING
    }
    override fun configure(): CommandBase {
        return ParallelCommandGroup(
                        SetWristAngles(robot.wrist, ArmStatePositionData.ARM_CONE_RIGHTING.wrist.bendAngle, ArmStatePositionData.ARM_CONE_RIGHTING.wrist.twistAngle),
                        SetExtensionLinkage(robot.extension, ArmStatePositionData.ARM_CONE_RIGHTING.extension.length, Cons.EASING),
                        SetAligner(robot.aligner, ArmStatePositionData.ARM_CONE_RIGHTING.aligner.angle),
                        SetElbowAngle(robot.elbow, ArmStatePositionData.ARM_CONE_RIGHTING.elbow.angle),
                )
    }
}