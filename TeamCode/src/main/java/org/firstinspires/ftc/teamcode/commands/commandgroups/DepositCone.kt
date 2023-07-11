package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.Cons
import org.firstinspires.ftc.teamcode.commands.SetElbowTarget
import org.firstinspires.ftc.teamcode.commands.extension.SetLinearExtension
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ExtensionLinkageSubsystem
import org.firstinspires.ftc.teamcode.subsystems.Robot

class DepositCone(val robot : Robot) : SequentialCommandGroup() {
    var earlyExit = false
    init {
        val wristTwistAngleDegrees = 0.0

        when(robot.armState) {
            ArmState.HIGH -> {
                val wristBendAngleDegrees = 21.0
                addCommands(SetWristAngles(robot.wrist, wristBendAngleDegrees, wristTwistAngleDegrees),)
            }
            ArmState.MED -> {
                val wristBendAngleDegrees = 4.0
                addCommands(SetWristAngles(robot.wrist, wristBendAngleDegrees, wristTwistAngleDegrees),)
            }
            ArmState.LOW -> {
                val wristBendAngleDegrees = -5.0
                addCommands(SetWristAngles(robot.wrist, wristBendAngleDegrees, wristTwistAngleDegrees),)
            }
            ArmState.GROUND -> {
                val wristBendAngleDegrees = -20.0
                addCommands(SetWristAngles(robot.wrist, wristBendAngleDegrees, wristTwistAngleDegrees),)
            }
            else -> {
                earlyExit = true
            }
        }
        addCommands(InstantCommand( {robot.claw.openClaw() }, robot.claw))
        addCommands(ParallelCommandGroup(
            InstantCommand( { robot.armState = ArmState.TRAVEL }, robot.elbow),
            SetAligner(robot.aligner, 0.0),
            SetLinearExtension(robot.extension, ExtensionLinkageSubsystem.MINIMUM_EXTENSION)))
        addCommands(SequentialCommandGroup(
            SetTurretAngle(robot.turret, 180.0),
            SetElbowTarget(robot, -50.0),
            SetWristAngles(robot.wrist, -120.0, 0.0)
        ))
    }

    override fun initialize() {
        if(!earlyExit)
            super.initialize()
    }
    override fun isFinished(): Boolean {
        return super.isFinished() || earlyExit
    }
}