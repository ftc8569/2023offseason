package org.firstinspires.ftc.teamcode.opmodes.auto.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.Cons
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

class IntakeFromConeStack(val robot : Robot, val alliancePosition: AlliancePosition, val coneNumber : Int): ConfigurableCommandBase()  {
    override fun configure() : CommandBase {
        val turretState = TurretStateData(92.0)
        val armAndTurretState = when (alliancePosition) {

            AlliancePosition.RIGHT -> when (coneNumber) {

                5 -> ArmAndTurretStateData(
                    ArmStateData(
                        WristStateData(-22.0, 0.0, 0.0),
                        ElbowStateData(-19.0),
                        ExtensionStateData(11.5),
                        PoleAlignerStateData(-90.0),
                        ArmStatePositionData.CLAW_OPEN_FOR_INTAKE
                    ),
                    turretState
                )

                4 -> ArmAndTurretStateData(
                    ArmStateData(
                        WristStateData(-26.0, 0.0, 0.0),
                        ElbowStateData(-22.0),
                        ExtensionStateData(11.75),
                        PoleAlignerStateData(-90.0),
                        ArmStatePositionData.CLAW_OPEN_FOR_INTAKE
                    ),
                    turretState
                )

                3 -> ArmAndTurretStateData(
                    ArmStateData(
                        WristStateData(-23.0, 0.0, 0.0),
                        ElbowStateData(-26.0),
                        ExtensionStateData(12.0),
                        PoleAlignerStateData(-90.0),
                        ArmStatePositionData.CLAW_OPEN_FOR_INTAKE
                    ),
                    turretState
                )

                2 -> ArmAndTurretStateData(
                    ArmStateData(
                        WristStateData(-22.0, 0.0, 0.0),
                        ElbowStateData(-27.0),
                        ExtensionStateData(12.5),
                        PoleAlignerStateData(-90.0),
                        ArmStatePositionData.CLAW_OPEN_FOR_INTAKE
                    ),
                    turretState
                )

                1 -> ArmAndTurretStateData(
                    ArmStateData(
                        WristStateData(-26.0, 0.0, 0.0),
                        ElbowStateData(-28.0),
                        ExtensionStateData(13.0),
                        PoleAlignerStateData(-90.0),
                        ArmStatePositionData.CLAW_OPEN_FOR_INTAKE
                    ),
                    turretState
                )

                else -> throw IllegalStateException()
            }

            AlliancePosition.CENTER_RIGHT, AlliancePosition.CENTER_LEFT, AlliancePosition.LEFT -> throw NotImplementedError()
            else -> throw IllegalStateException()
        }

        val extensionPickupOffset = 1.0
        val elbowAnglePickupOffset = 15.0

        return SequentialCommandGroup(
            ParallelCommandGroup(
                SetAligner(robot.aligner, armAndTurretState.arm.aligner.angle),
                SetClawPosition(robot.claw, ClawPositions.OPEN_FOR_INTAKE)
            ),
            SetTurretAngle(robot.turret, armAndTurretState.turret.angle),
            SetElbowAngle(robot.elbow, armAndTurretState.arm.elbow.angle),
            SetExtensionLinkage(robot.extension, armAndTurretState.arm.extension.length, EASING),
            SetWristAngles(robot.wrist, armAndTurretState.arm.wrist.bendAngle, armAndTurretState.arm.wrist.twistAngle),
            WaitCommand(Cons.COMMAND_DELAY),

            CloseClawOnBeamBreak(robot),
            SetElbowAngle(robot.elbow, armAndTurretState.arm.elbow.angle + elbowAnglePickupOffset),
            SetExtensionLinkage(robot.extension, armAndTurretState.arm.extension.length - extensionPickupOffset),
            WaitCommand(100),
            MoveToTravel(robot)
        )

    }
}