package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.commands.general.UpdateTelemetry
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.*

class DepositCone(val robot : Robot) : ConfigurableCommandBase()  {


    var has_initialized = false

    override fun configure(): CommandBase {
        val canDepositCone = when(robot.armState) {
            ArmState.HIGH, ArmState.MED, ArmState.LOW, ArmState.GROUND -> true
            else -> false
        }
        has_initialized = true
        if(!canDepositCone)
            return UpdateTelemetry(robot) { telemetry ->
                telemetry.addLine("DepositCone is not configured for arm state ${robot.armState}")
            }
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
                InstantCommand({ robot.claw.position = ClawPositions.OPEN_FOR_INTAKE }, robot.claw),
                WaitCommand(500),
                MoveToTravel(robot)
            )
        }

    }

    override fun isFinished(): Boolean {
        return super.isFinished() && has_initialized
    }

}