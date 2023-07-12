package org.firstinspires.ftc.teamcode.commands.drivetrain

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.Cons.HEADING_KD
import org.firstinspires.ftc.teamcode.Cons.HEADING_KP
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem

class DriveMecanumSnap (private val drive: DrivetrainSubsystem,
                        private val goalHeading:Double,
                        private val fwdSupplier: () -> Double,
                        private val strafeSupplier: () -> Double) : CommandBase() {

    val pid_basic = BasicPID(PIDCoefficients(HEADING_KP, 0.0, HEADING_KD))
    val angle_controller = AngleController(pid_basic)

    init {
        addRequirements(drive)
    }

    override fun execute() {
        val out = angle_controller.calculate(goalHeading,drive.poseEstimate.heading)

        drive.driveFieldCentric(
            strafeSupplier(),
            fwdSupplier(),
            -out,
            drive.poseEstimate.heading
        )
    }
}