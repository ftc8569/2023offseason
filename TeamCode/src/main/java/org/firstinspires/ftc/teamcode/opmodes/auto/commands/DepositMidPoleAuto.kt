package org.firstinspires.ftc.teamcode.opmodes.auto.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.Cons
import org.firstinspires.ftc.teamcode.commands.claw.ClawRegripCone
import org.firstinspires.ftc.teamcode.commands.claw.SetClawPosition
import org.firstinspires.ftc.teamcode.commands.commandgroups.MinimalStowFromPoleToStack
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmAndTurretStateData
import org.firstinspires.ftc.teamcode.subsystems.ArmStateData
import org.firstinspires.ftc.teamcode.subsystems.ArmStatePositionData
import org.firstinspires.ftc.teamcode.subsystems.ArmStatePositionData.Companion.ARM_HOME
import org.firstinspires.ftc.teamcode.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.subsystems.ElbowStateData
import org.firstinspires.ftc.teamcode.subsystems.ExtensionStateData
import org.firstinspires.ftc.teamcode.subsystems.PoleAlignerStateData
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.subsystems.TurretStateData
import org.firstinspires.ftc.teamcode.subsystems.WristStateData

class DepositMidPoleAuto(
        val robot: Robot, val alliancePosition: AlliancePosition) : ConfigurableCommandBase() {
    override fun configure(): CommandBase {
            val armAndTurretState = when (alliancePosition) {
                AlliancePosition.LEFT -> ArmAndTurretStateData(
                        ArmStateData(
                                WristStateData(-5.0, 90.0, 25.0, 0.0),
                                ElbowStateData(25.8),
                                ExtensionStateData(4.6),
                                PoleAlignerStateData(ARM_HOME.aligner.angle),
                                ArmStatePositionData.CLAW_HOLD_CONE
                        ),
                        TurretStateData(-251.0) // -251.0
                )
                AlliancePosition.RIGHT -> ArmAndTurretStateData(
                        ArmStateData(
                                WristStateData(-5.0, -90.0, 25.0, 0.0),
                                ElbowStateData(25.8),
                                ExtensionStateData(4.6),
                                PoleAlignerStateData(ARM_HOME.aligner.angle),
                                ArmStatePositionData.CLAW_HOLD_CONE
                        ),
                        TurretStateData(251.0) // -251.0
                )

                AlliancePosition.CENTER_RIGHT, AlliancePosition.CENTER_LEFT -> throw NotImplementedError()
                else -> throw IllegalStateException()
            }


            return SequentialCommandGroup(
                    ParallelCommandGroup(
                            SetElbowAngle(robot.elbow, armAndTurretState.arm.elbow.angle),
                            SetTurretAngle(robot.turret, armAndTurretState.turret.angle),
                            SetExtensionLinkage(robot.extension, armAndTurretState.arm.extension.length),
                            SetWristAngles(robot.wrist, armAndTurretState.arm.wrist.bendAngle, armAndTurretState.arm.wrist.twistAngle)
                            ),
                    WaitCommand(375),
                    SetWristAngles(robot.wrist, armAndTurretState.arm.wrist.depositBendAngle, armAndTurretState.arm.wrist.depositTwistAngle),
                    SetClawPosition(robot.claw, ClawPositions.OPEN_FOR_INTAKE),
                    ParallelCommandGroup(
                    SetWristAngles(robot.wrist, -60.0, 0.0),
                    SetElbowAngle(robot.elbow, armAndTurretState.arm.elbow.angle+10.0)
                    ),

//                    MinimalStowFromPoleToStack(robot)
            )
    }
}

