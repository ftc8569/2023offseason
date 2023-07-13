package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.*
import org.firstinspires.ftc.teamcode.commands.claw.SetClawPosition
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.commands.general.UpdateTelemetry
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.ExtensionLinkageSubsystem
import org.firstinspires.ftc.teamcode.subsystems.Robot

class PickupCone(private val robot : Robot) : ConfigurableCommandBase() {

    override fun configure(): CommandBase {
        return if (robot.armState == ArmState.INTAKE) {
            SequentialCommandGroup(
                SetClawPosition(robot.claw, ClawPositions.HOLD_CONE),
                MoveToTravel(robot)
            )
        } else
            UpdateTelemetry(robot) { telemetry ->
                telemetry.addLine("PickupCone is not configured for arm state ${robot.armState}")
            }
    }
}