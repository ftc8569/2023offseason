package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SelectCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.commands.UpdateTelemetry
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmAndTurretStateData
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.Robot

class IntakeCone(private var robot : Robot, newState: ArmAndTurretStateData) : SelectCommand({ generateCommand(robot, newState) }) {

    override fun initialize() {
        super.initialize()
        robot.armState = ArmState.INTAKE
    }

    companion object {
        fun generateCommand(robot : Robot, newState: ArmAndTurretStateData): CommandBase {
            // We can only enter intake mode from TRAVEL mode
            // TODO check that the armStateData passes in is legit & consider ParalllelCommandGroup
            return if (robot.armState == ArmState.TRAVEL) {
                SequentialCommandGroup(
                    SetAligner(robot.aligner, newState.arm.aligner.angle),
                    InstantCommand({robot.claw.openClaw()}, robot.claw),
                    SetWristAngles(robot.wrist, newState.arm.wrist.bendAngle, newState.arm.wrist.twistAngle),
                    SetElbowAngle(robot.elbow, newState.arm.elbow.angle),
                    SetExtensionLinkage(robot.extension, newState.arm.extension.length)
                )

            } else {
                UpdateTelemetry(robot, "Cannot move to intake from arm state ${robot.armState} - must be in TRAVEL")
            }
        }
    }
}