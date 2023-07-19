package org.firstinspires.ftc.teamcode.commands.drivetrain

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem

class ResetHeading(val drive: DrivetrainSubsystem): CommandBase() {
    override fun initialize() {
        addRequirements(drive)
        drive.resetHeading()
    }

    override fun isFinished() = true

}