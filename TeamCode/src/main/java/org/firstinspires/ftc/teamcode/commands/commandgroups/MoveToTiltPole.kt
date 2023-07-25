package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ArmStatePositionData
import org.firstinspires.ftc.teamcode.subsystems.Robot

class MoveToTiltPole (val robot : Robot) : ConfigurableCommandBase() {

    override fun initialize() {
        super.initialize()
    }

    override fun configure(): CommandBase {
        val armState = ArmStatePositionData.TILT_POLE

        return SequentialCommandGroup(
                SetElbowAngle(robot.elbow, armState.elbow.angle),
                SetAligner(robot.aligner, armState.aligner.angle),
                ParallelCommandGroup(
                        SetExtensionLinkage(robot.extension, armState.extension.length),
                        SetWristAngles(robot.wrist, armState.wrist.bendAngle, armState.wrist.twistAngle)
                )
        )
    }
}