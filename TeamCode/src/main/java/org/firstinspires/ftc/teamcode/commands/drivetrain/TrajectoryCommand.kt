package org.firstinspires.ftc.teamcode.commands.drivetrain

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.subsystems.RRDrivetrain

class TrajectoryCommand(
    private val drive: RRDrivetrain,
    private val trajectory: TrajectorySequence
) :
    CommandBase() {

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        drive.followTrajectorySequenceAsync(trajectory)
    }

    override fun execute() {
        drive.update()
    }

    override fun isFinished(): Boolean {
        return !drive.isBusy
    }
}