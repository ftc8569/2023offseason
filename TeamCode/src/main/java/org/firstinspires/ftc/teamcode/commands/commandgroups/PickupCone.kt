package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.extension.SetLinearExtension
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ExtensionLinkageSubsystem
import org.firstinspires.ftc.teamcode.subsystems.Robot

class PickupCone(var robot : Robot) : SequentialCommandGroup() {

    init {
        if (robot.armState == ArmState.INTAKE) {
            robot.armState = ArmState.TRAVEL
            this.addCommands(
                InstantCommand({robot.claw.closeClaw()}, robot.claw),
                WaitCommand(150),
                ParallelCommandGroup(
                    SetWristAngles(robot.wrist, -18.0, 0.0),
                    SetLinearExtension(robot.extension, ExtensionLinkageSubsystem.MINIMUM_EXTENSION)
                )
            )
        }

    }
}