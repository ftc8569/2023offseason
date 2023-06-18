package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.Subsystem
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import kotlin.math.PI


/* This class is just an abstraction layer on top of Roadrunner's SampleMecanumDrive that is also
an FTCLib subsystem. In the regular season code, we had two drivetrain classes: one for auto that
used roadrunner, and one for teleop that used FTCLib. This is unnecessary, and makes it more
difficult to use RR odometry within TeleOp. Instead, we will be using RR exclusively. Please
forgive me for my sins -- Jack Fetkovich, 06/18/2023
* */
class RRDrivetrain(val hw: HardwareMap): SampleMecanumDrive(hw), Subsystem {
    fun driveFieldCentric(
        strafeSpeed: Double, forwardSpeed: Double,
        turnSpeed: Double, gyroAngle: Double
    ) {
        var drivePower = Vector2d(strafeSpeed, forwardSpeed)
        drivePower = drivePower.rotated(- gyroAngle * PI/180)
        val rotatedPower = Pose2d(drivePower, 0.0)
        setWeightedDrivePower(rotatedPower)
    }

    /* Everything from SubsystemBase class, implmenting Subsystem interface so that I can
     inherit the SampleMecanumDrive class */
    private var m_name = this.javaClass.simpleName
    fun SubsystemBase() {
        CommandScheduler.getInstance().registerSubsystem(this)
    }
    fun getName(): String? {
        return m_name
    }
    fun setName(name: String) {
        m_name = name
    }
    fun getSubsystem(): String? {
        return getName()
    }
    fun setSubsystem(subsystem: String) {
        setName(subsystem)
    }
}