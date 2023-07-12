package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SelectCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.UpdateTelemetry
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.*

class DepositCone(val robot : Robot) : SelectCommand({ generateCommand(robot) })  {
    companion object {
        fun generateCommand(robot: Robot): CommandBase {
            val canDepositCone = when(robot.armState) {
                ArmState.HIGH, ArmState.MED, ArmState.LOW, ArmState.GROUND -> true
                else -> false
            }

            if(!canDepositCone)
                return UpdateTelemetry(robot, "Cannot deposit cone with arm state ${robot.armState}")
            else {
                val wrist = when (robot.armState) {
                    ArmState.HIGH -> ArmStates.SCORE_HIGH.wrist
                    ArmState.MED -> ArmStates.SCORE_MIDDLE.wrist
                    ArmState.LOW -> ArmStates.SCORE_LOW.wrist
                    ArmState.GROUND -> ArmStates.SCORE_GROUND.wrist
                    else -> ArmStates.ARM_HOME.wrist
                    }

                return SequentialCommandGroup(
                    SetWristAngles(robot.wrist, wrist.depositBendAngle, 0.0),
                    InstantCommand({ robot.claw.openClaw() }, robot.claw),
                    WaitCommand(500),
                    MoveToTravel(robot)
                    )
            }
        }
    }
}