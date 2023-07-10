package org.firstinspires.ftc.teamcode.commands.drivetrain

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem

class DriveFieldCentric(
    private val dt: DrivetrainSubsystem,
    val strafeSpeed: () -> Double,
    val forwardSpeed: () -> Double,
    val turnSpeed: () -> Double,
    val gyroAngle: () -> Double
) : CommandBase() {
    init {
        addRequirements(dt)
    }

    override fun execute() {
        dt.driveFieldCentric(
            strafeSpeed.invoke(),
            forwardSpeed.invoke(),
            turnSpeed.invoke(),
            gyroAngle.invoke()
        )
    }


    override fun isFinished() = false
}