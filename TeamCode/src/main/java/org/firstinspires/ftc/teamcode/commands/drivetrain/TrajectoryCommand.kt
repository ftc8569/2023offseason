package org.firstinspires.ftc.teamcode.commands.drivetrain

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.subsystems.RRDrivetrain

class TrajectoryCommand(
    private val autodrive: RRDrivetrain,
    private val trajectory: TrajectorySequence
) :
    CommandBase() {

    init {
        addRequirements(autodrive)
    }

    override fun initialize() {
        autodrive.followTrajectorySequenceAsync(trajectory)
    }

    override fun execute() {
        autodrive.update()
    }

    override fun isFinished(): Boolean {
        return !autodrive.isBusy
    }
}