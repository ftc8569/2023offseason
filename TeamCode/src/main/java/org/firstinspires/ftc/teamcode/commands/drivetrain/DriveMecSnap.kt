package org.firstinspires.ftc.teamcode.commands.drivetrain

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.teamcode.Cons.HEADING_KD
import org.firstinspires.ftc.teamcode.Cons.HEADING_KP
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain

class DriveMecSnap(
    val drive: Drivetrain,private val goalHeading:Double, private val fwdSupplier: () -> Double,
    private val strafeSupplier: () -> Double,
) : CommandBase() {
    val controller = PIDController(HEADING_KP, 0.0, HEADING_KD)

    init {
        addRequirements(drive)
    }

    override fun execute() {
        val out = controller.calculate(drive.getYaw(), goalHeading)

        drive.drive.driveFieldCentric(
            strafeSupplier.invoke(),
            fwdSupplier.invoke(),
            -out,
            drive.getYaw()
        )
    }
}