package org.firstinspires.ftc.teamcode.commands.drivetrain

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain

class DriveMec(
    val drive: Drivetrain,
    private val fwdSupplier: () -> Double,
    private val strafeSupplier: () -> Double,
    private val turnSupplier: () -> Double
) : CommandBase() {

    init {
        addRequirements(drive)
    }

    override fun execute() {
        drive.drive.driveFieldCentric(
            strafeSupplier.invoke(),
            fwdSupplier.invoke(),
            turnSupplier.invoke(),
            drive.getYaw()
        )
    }


}