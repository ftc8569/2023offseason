package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import org.firstinspires.ftc.teamcode.commands.SetElbowTarget
import org.firstinspires.ftc.teamcode.commands.extension.SetLinearExtension
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.Robot

class ScoreLowJunction(val robot : Robot) : ParallelCommandGroup() {
    init {
        val wristBendAngleDegrees = -26.0
        val wristTwistAngleDegrees = 0.0
        val elbowAngleDegrees = 18.25
        val extensionLengthInches = 0.0
        val poleAlignerPosition = 0.8

        this.addCommands(
            SetWristAngles(robot.wrist, wristBendAngleDegrees, wristTwistAngleDegrees),
            SetElbowTarget(robot,elbowAngleDegrees),
            SetLinearExtension(robot.extension, extensionLengthInches),
            SetAligner(robot.aligner, poleAlignerPosition)
        )
    }
    override fun initialize() {
        super.initialize()
        robot.armState = ArmState.LOW
    }

}
