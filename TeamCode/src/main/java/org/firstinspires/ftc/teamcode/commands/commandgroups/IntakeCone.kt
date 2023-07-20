package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SelectCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.commands.claw.SetClawPosition
import org.firstinspires.ftc.teamcode.commands.general.UpdateTelemetry
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.*

class IntakeCone(private var robot : Robot, private val newState: ArmStateData) : ConfigurableCommandBase() {

    override fun initialize() {
        super.initialize()
        robot.armState = ArmState.INTAKE
    }

    override fun configure(): CommandBase {
        // We can only enter intake mode from TRAVEL mode
        // TODO check that the armStateData passes in is legit & consider ParallelCommandGroup
        return if (robot.armState == ArmState.TRAVEL) {
            SequentialCommandGroup(
                SetAligner(robot.aligner, newState.aligner.angle),
                SetElbowAngle(robot.elbow, newState.elbow.angle),
                ParallelCommandGroup(
                    SetClawPosition(robot.claw, ClawPositions.OPEN_FOR_INTAKE),
                    SetWristAngles(robot.wrist, newState.wrist.bendAngle, newState.wrist.twistAngle),
                    SetExtensionLinkage(robot.extension, newState.extension.length),
                    )
                )

        } else {
            UpdateTelemetry(robot) { telemetry ->
                telemetry.addLine("Cannot move to intake from arm state ${robot.armState} - must be in TRAVEL")
            }
        }
    }
}