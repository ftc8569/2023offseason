package org.firstinspires.ftc.teamcode.commands.drivetrain

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.Cons.HEADING_KD
import org.firstinspires.ftc.teamcode.Cons.HEADING_KP
import org.firstinspires.ftc.teamcode.roadrunner.drive.JuicyMecanumDrive.TRANSLATIONAL_PID
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem

class DriveMecanumSnap (private val drive: DrivetrainSubsystem,
                        private val goalHeading:Double,
                        private val fwdSupplier: () -> Double,
                        private val strafeSupplier: () -> Double) : CommandBase() {

    val headingPID = BasicPID(PIDCoefficients(HEADING_KP, 0.0, HEADING_KD))
    val angle_controller = AngleController(headingPID)
    var isHeadingLocked = true
    private var headingAdjustment = 0.0

    val laneKeepingPID = BasicPID(PIDCoefficients(TRANSLATIONAL_PID.kP, TRANSLATIONAL_PID.kI, TRANSLATIONAL_PID.kD))

    init {
        addRequirements(drive)
    }

    fun resetHeadingToImuHeading() {
        var newHeading = drive.rawExternalHeading
        drive.poseEstimate = drive.poseEstimate.copy(heading = newHeading)
    }
    fun adjustHeading(headingAdjustmentDegrees: Double) {
        headingAdjustment += Math.toRadians(headingAdjustmentDegrees)
//        var newHeading = drive.poseEstimate.heading + Math.toRadians(headingAdjustmentDegrees)
//        drive.poseEstimate = drive.poseEstimate.copy(heading = newHeading)
    }
    fun resetGoalHeadingToCurrentHeading() {
        drive.poseEstimate = drive.poseEstimate.copy(heading = goalHeading)
    }
    override fun execute() {
        val headingToUse = drive.rawExternalHeading + headingAdjustment
        val angleControllerCorrection = if(isHeadingLocked) angle_controller.calculate(goalHeading, headingToUse) else 0.0

        drive.driveFieldCentric(
            strafeSupplier(),
            fwdSupplier(),
            -angleControllerCorrection,
            drive.poseEstimate.heading
        )
    }
}