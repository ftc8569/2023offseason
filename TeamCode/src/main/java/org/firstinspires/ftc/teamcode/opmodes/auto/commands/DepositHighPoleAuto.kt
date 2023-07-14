package org.firstinspires.ftc.teamcode.opmodes.auto.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.Cons.EASING
import org.firstinspires.ftc.teamcode.commands.claw.SetClawPosition
import org.firstinspires.ftc.teamcode.commands.commandgroups.MoveToTravel
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmAndTurretStateData
import org.firstinspires.ftc.teamcode.subsystems.ArmStateData
import org.firstinspires.ftc.teamcode.subsystems.ArmStatePositionData
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.ElbowStateData
import org.firstinspires.ftc.teamcode.subsystems.ExtensionStateData
import org.firstinspires.ftc.teamcode.subsystems.PoleAlignerStateData
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.subsystems.TurretStateData
import org.firstinspires.ftc.teamcode.subsystems.WristStateData

class DepositHighPoleAuto(val robot : Robot, val alliancePosition: AlliancePosition) : ConfigurableCommandBase() {
    override fun configure(): CommandBase {
        val armAndTurretState = when (alliancePosition) {
            AlliancePosition.RIGHT -> ArmAndTurretStateData(ArmStateData(
                WristStateData(-12.0, 0.0, 18.0),
                ElbowStateData(50.0),
                ExtensionStateData(12.5),
                PoleAlignerStateData(21.0),
                ArmStatePositionData.CLAW_HOLD_CONE),
                TurretStateData(-53.2))

            AlliancePosition.CENTER_RIGHT, AlliancePosition.CENTER_LEFT, AlliancePosition.LEFT -> throw NotImplementedError()
            else -> throw IllegalStateException()
        }
        return SequentialCommandGroup(
            SetTurretAngle(robot.turret, armAndTurretState.turret.angle),
            SetElbowAngle(robot.elbow, armAndTurretState.arm.elbow.angle),
            SetAligner(robot.aligner, armAndTurretState.arm.aligner.angle),
            SetExtensionLinkage(robot.extension, armAndTurretState.arm.extension.length, EASING),
            WaitCommand(250),
            SetWristAngles(robot.wrist, armAndTurretState.arm.wrist.depositBendAngle, 0.0),
            SetClawPosition(robot.claw, ClawPositions.OPEN_FOR_INTAKE),
            SetAligner(robot.aligner, ArmStatePositionData.ARM_HOME.aligner.angle),
            MoveToTravel(robot)
        )
    }
}