package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import org.firstinspires.ftc.teamcode.commands.SetElbowTarget
import org.firstinspires.ftc.teamcode.commands.extension.SetLinearExtension
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.Robot

class ScoreHighJunction(val robot : Robot) : ParallelCommandGroup() {
    init {
        val wristBendAngleDegrees = 1.0
        val wristTwistAngleDegrees = 0.0
        val elbowAngleDegrees = 57.0
        val extensionLengthInches = 14.0
        val poleAlignerPosition = 0.5

        this.addCommands(
            SetWristAngles(robot.wrist, wristBendAngleDegrees, wristTwistAngleDegrees),
            SetElbowTarget(robot,elbowAngleDegrees),
            SetLinearExtension(robot.extension, extensionLengthInches),
            SetAligner(robot.aligner, poleAlignerPosition)
        )
    }
    override fun initialize() {
        super.initialize()
        robot.armState = ArmState.HIGH
    }
}