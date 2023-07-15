package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.claw.SetClawPosition
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.commands.general.UpdateTelemetry
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ArmStatePositionData
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.Robot

class DepositTSEUnderCone(val robot : Robot) : ConfigurableCommandBase()  {

    override fun configure(): CommandBase {
        val canDepositCone = when(robot.armState) {
            ArmState.SCORE_HIGH, ArmState.SCORE_MEDIUM, ArmState.SCORE_LOW, ArmState.SCORE_GROUND -> true
            else -> false
        }

        if(!canDepositCone)
            return UpdateTelemetry(robot) { telemetry ->
                telemetry.addLine("DepositCone is not configured for arm state ${robot.armState}")
            }
        else {
            val wrist = when (robot.armState) {
                ArmState.SCORE_HIGH -> ArmStatePositionData.SCORE_HIGH.wrist
                ArmState.SCORE_MEDIUM -> ArmStatePositionData.SCORE_MEDIUM.wrist
                ArmState.SCORE_LOW -> ArmStatePositionData.SCORE_LOW.wrist
                ArmState.SCORE_GROUND -> ArmStatePositionData.SCORE_GROUND.wrist
                else -> ArmStatePositionData.ARM_HOME.wrist
            }

            return SequentialCommandGroup(
                SequentialCommandGroup(
                    SetWristAngles(robot.wrist, wrist.depositBendAngle, 0.0),
                    SetClawPosition(robot.claw, ClawPositions.RELEASE_CONE_BUT_HOLD_TSE),
                ),
                ParallelCommandGroup(
                    SetAligner(robot.aligner, ArmStatePositionData.ARM_HOME.aligner.angle),
                    WaitCommand(250),
                ),
                MoveToTravel(robot)
            )
        }
    }
}