package org.firstinspires.ftc.teamcode.commands.drivetrain

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.subsystems.AutoDrive
import org.firstinspires.ftc.teamcode.subsystems.RRDrivetrain

class TrajectoryCommand(
    private val autodrive: AutoDrive,
    private val trajectory: TrajectorySequence
) :
    CommandBase() {

    init {
        addRequirements(autodrive)
    }

    override fun initialize() {
        autodrive.drive.followTrajectorySequenceAsync(trajectory)
    }

    override fun execute() {
        autodrive.drive.update()
    }

    override fun isFinished(): Boolean {
        return !autodrive.drive.isBusy
    }
}