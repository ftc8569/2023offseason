package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SelectCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.commands.general.UpdateTelemetry
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ArmStates
import org.firstinspires.ftc.teamcode.subsystems.Robot

open class ScoreJunction(private val robot : Robot, private val targetState : ArmState) : ConfigurableCommandBase() {
    override fun initialize() {
        super.initialize()
        robot.armState = targetState
    }

    override fun configure(): CommandBase {
        val choice = when(targetState) {
            ArmState.HIGH -> Triple(true, ArmStates.SCORE_HIGH, true)
            ArmState.MED -> Triple(true, ArmStates.SCORE_MIDDLE,true)
            ArmState.LOW -> Triple(true, ArmStates.SCORE_LOW, true)
            ArmState.GROUND -> Triple(true, ArmStates.SCORE_GROUND, true)
            else -> Triple(false, ArmStates.ARM_HOME, false)
        }

        val shouldElbowGoFirst = choice.first
        val armState = choice.second
        val isArmStateValid = choice.third

        if(!isArmStateValid)
            return UpdateTelemetry(robot) {
                robot.telemetry.addLine("Cannot score to arm state $targetState")
            }

        if (shouldElbowGoFirst)
            return SequentialCommandGroup(
                SetElbowAngle(robot.elbow, armState.elbow.angle),
                ParallelCommandGroup(
                    SetWristAngles(robot.wrist, armState.wrist.bendAngle, armState.wrist.twistAngle),
                    SetExtensionLinkage(robot.extension, armState.extension.length),
                    SetAligner(robot.aligner, armState.aligner.angle)
                ))
        else
            return SequentialCommandGroup(
                ParallelCommandGroup(
                    SetWristAngles(robot.wrist, armState.wrist.bendAngle, armState.wrist.twistAngle),
                    SetExtensionLinkage(robot.extension, armState.extension.length),
                    SetAligner(robot.aligner, armState.aligner.angle)
                ),
                SetElbowAngle(robot.elbow, armState.elbow.angle))
    }

}