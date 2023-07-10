package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.Subsystem
import com.arcrobotics.ftclib.util.MathUtils.clamp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.drive.JuicyMecanumDrive
import kotlin.math.abs


/* This class is just an abstraction layer on top of Roadrunner's SampleMecanumDrive that is also
an FTCLib subsystem. In the regular season code, we had two drivetrain classes: one for auto that
used roadrunner, and one for teleop that used FTCLib. This is unnecessary, and makes it more
difficult to use RR odometry within TeleOp. Instead, we will be using RR exclusively. Please
forgive me for my sins -- Jack Fetkovich, 06/18/2023

Granted - Carl & Ben 7/9/2023
* */
class DrivetrainSubsystem(val robot : Robot) : JuicyMecanumDrive(robot.hardwareMap), Subsystem {
    fun driveFieldCentric(strafeSpeed: Double, forwardSpeed: Double, turnSpeed: Double, heading: Double) {
        val strafeSpeed = clamp(strafeSpeed, -1.0, 1.0)
        val forwardSpeed = clamp(forwardSpeed, -1.0, 1.0)
        val turnSpeed = clamp(turnSpeed, -1.0, 1.0)

        var input = com.arcrobotics.ftclib.geometry.Vector2d(forwardSpeed, strafeSpeed)
        input = input.rotateBy(Math.toDegrees(heading))

        setWeightedDrivePower(Pose2d(input.x, -input.y, -turnSpeed))
    }

    /* Everything from SubsystemBase class, implmenting Subsystem interface so that I can
     inherit the SampleMecanumDrive class */
    var m_name = this.javaClass.simpleName
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

    private fun normalize(arr: DoubleArray): DoubleArray {
        var normalizedList = DoubleArray(4)
        var maxMagnitude: Double = abs(arr[0])
        for (i in 1 until arr.size) {
            val temp: Double = abs(arr[i])
            if (maxMagnitude < temp) {
                maxMagnitude = temp
            }
        }
        if (maxMagnitude > 1) {
            for (i in arr.indices) {
                normalizedList[i] = arr[i] / maxMagnitude
            }
        }
//        else {
//            for (i in arr.indices) {
//                normalizedList[i] = (arr[i])
//            }
//        }
        return normalizedList
    }

    private fun normalize(arr: DoubleArray, magnitude: Double): DoubleArray {
        var normalizedList = DoubleArray(4)
        var maxMagnitude: Double = abs(arr[0])
        for (i in 1 until arr.size) {
            val temp: Double = abs(arr[i])
            if (maxMagnitude < temp) {
                maxMagnitude = temp
            }
        }

        for (i in arr.indices) {
            normalizedList[i] = (arr[i] / maxMagnitude) * magnitude
        }

        return normalizedList
    }

     fun resetHeading() {
        poseEstimate = Pose2d(poseEstimate.x,poseEstimate.y,0.0)
    }

    override fun periodic() {
        update()
    }

}