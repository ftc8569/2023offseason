package org.firstinspires.ftc.teamcode.opmodes.auto.commands

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.Cons
import org.firstinspires.ftc.teamcode.commands.drivetrain.TrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.roadrunner.drive.JuicyMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Robot

class HoldRobotPose(val robot : Robot, val poseToHold : Pose2d) : CommandBase() {
    val headingPID = BasicPID(PIDCoefficients(Cons.HEADING_KP, 0.0, Cons.HEADING_KD))
    val angle_controller = AngleController(headingPID)
    val drive = robot.drivetrain
    val xPID = BasicPID(PIDCoefficients(JuicyMecanumDrive.TRANSLATIONAL_PID.kP, JuicyMecanumDrive.TRANSLATIONAL_PID.kI, JuicyMecanumDrive.TRANSLATIONAL_PID.kD))
    val yPID = BasicPID(PIDCoefficients(JuicyMecanumDrive.TRANSLATIONAL_PID.kP, JuicyMecanumDrive.TRANSLATIONAL_PID.kI, JuicyMecanumDrive.TRANSLATIONAL_PID.kD))

    init {
        addRequirements(drive)
    }

    override fun execute() {
        val headingToUse = drive.poseEstimate.heading
        val angleControllerCorrection = angle_controller.calculate(poseToHold.heading, headingToUse)
        val strafe = xPID.calculate(poseToHold.x, drive.poseEstimate.x )
        val forward = yPID.calculate(poseToHold.y, drive.poseEstimate.y)
        println("HEADING = " +headingToUse)
        drive.driveFieldCentric(0.0, 0.0, -angleControllerCorrection, headingToUse)
    }
    override fun isFinished(): Boolean {
        return false
    }


}