package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.commands.SetElbowTarget
import org.firstinspires.ftc.teamcode.commands.extension.SetLinearExtension
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.Robot

class TeleopIntakeCone(private var robot : Robot) : SequentialCommandGroup() {
    init {
        val wristBendAngleDegrees = -18.0
        val wristTwistAngleDegrees = 0.0
        val elbowAngleDegrees = -40.0
        val extensionLengthInches = 11.75


        if (robot.armState == ArmState.TRAVEL) {
            robot.armState = ArmState.INTAKE
            this.addCommands(
                InstantCommand({robot.claw.openClaw()}, robot.claw),
                SetWristAngles(robot.wrist, wristBendAngleDegrees, wristTwistAngleDegrees),
                SetElbowTarget(robot,elbowAngleDegrees),
                SetLinearExtension(robot.extension, extensionLengthInches)
            )
        }

    }
}