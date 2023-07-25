package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ArmStatePositionData
import org.firstinspires.ftc.teamcode.subsystems.Robot

class MoveToCycle (val robot : Robot) : ConfigurableCommandBase() {

    override fun initialize() {
        super.initialize()
        robot.armState = ArmState.SCORE_HIGH
    }

    override fun configure(): CommandBase {
        val armState = ArmStatePositionData.SCORE_HIGH

        return SequentialCommandGroup(
                SetTurretAngle(robot.turret, 0.0),
                SetElbowAngle(robot.elbow, armState.elbow.angle),
                SetWristAngles(robot.wrist, armState.wrist.bendAngle, armState.wrist.twistAngle),
                WaitCommand(75),
                SetAligner(robot.aligner, armState.aligner.angle),
                WaitCommand(75),
                SetExtensionLinkage(robot.extension, armState.extension.length),
                WaitCommand(450),
                SetWristAngles(robot.wrist, armState.wrist.depositBendAngle, armState.wrist.twistAngle),
                DepositCone(robot),
                WaitCommand(25),
                MoveToTravel(robot),
                SetTurretAngle(robot.turret, 180.0)

        )
    }
}