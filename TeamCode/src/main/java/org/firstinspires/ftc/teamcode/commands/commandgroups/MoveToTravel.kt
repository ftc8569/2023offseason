package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SelectCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmAndTurretStateData
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ArmStates
import org.firstinspires.ftc.teamcode.subsystems.Robot

class MoveToTravel(val robot : Robot) : ConfigurableCommandBase()  {
    override fun initialize() {
        super.initialize()
        robot.armState = ArmState.TRAVEL
    }

    private val LowerElbowFirst = SequentialCommandGroup(
            SetElbowAngle(robot.elbow, ArmStates.TRAVEL.elbow.angle),
            ParallelCommandGroup(
                SetWristAngles(
                    robot.wrist,
                    ArmStates.TRAVEL.wrist.bendAngle,
                    ArmStates.TRAVEL.wrist.twistAngle
                ),
                SetExtensionLinkage(robot.extension, ArmStates.TRAVEL.extension.length),
                SetAligner(robot.aligner, ArmStates.TRAVEL.aligner.angle)
            )
        )
    override fun configure(): CommandBase {
        if (robot.armState == ArmState.INTAKE || robot.armState == ArmState.GROUND) {
            return SequentialCommandGroup(
            )
        } else {
            return SequentialCommandGroup(
                ParallelCommandGroup(
                    SetWristAngles(
                        robot.wrist,
                        ArmStates.TRAVEL.wrist.bendAngle,
                        ArmStates.TRAVEL.wrist.twistAngle
                    ),
                    SetExtensionLinkage(robot.extension, ArmStates.TRAVEL.extension.length),
                    SetAligner(robot.aligner, ArmStates.TRAVEL.aligner.angle)
                ),
                SetElbowAngle(robot.elbow, ArmStates.TRAVEL.elbow.angle)
            )
        }
    }
}