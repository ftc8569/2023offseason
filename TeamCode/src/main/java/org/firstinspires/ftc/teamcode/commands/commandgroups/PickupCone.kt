package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ExtensionLinkageSubsystem
import org.firstinspires.ftc.teamcode.subsystems.Robot

class PickupCone(var robot : Robot) : SequentialCommandGroup() {

    init {
        if (robot.armState == ArmState.INTAKE) {
            this.addCommands(
                InstantCommand({robot.claw.closeClaw()}, robot.claw),
                WaitCommand(150),
                MoveToTravel(robot)
            )
        }
    }
}