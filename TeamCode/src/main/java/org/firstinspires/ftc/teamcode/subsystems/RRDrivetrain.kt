package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.Subsystem
import com.arcrobotics.ftclib.util.MathUtils.clamp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import kotlin.math.abs


/* This class is just an abstraction layer on top of Roadrunner's SampleMecanumDrive that is also
an FTCLib subsystem. In the regular season code, we had two drivetrain classes: one for auto that
used roadrunner, and one for teleop that used FTCLib. This is unnecessary, and makes it more
difficult to use RR odometry within TeleOp. Instead, we will be using RR exclusively. Please
forgive me for my sins -- Jack Fetkovich, 06/18/2023
* */
class RRDrivetrain(val hw: HardwareMap, val robot: Robot) : SampleMecanumDrive(hw), Subsystem {
    fun driveFieldCentric(
        strafeSpeed: Double, forwardSpeed: Double,
        turnSpeed: Double, gyroAngle: Double
    ) {
        var strafeSpeed = clamp(strafeSpeed, -1.0, 1.0)
        var forwardSpeed = clamp(forwardSpeed, -1.0, 1.0)
        var turnSpeed = clamp(turnSpeed, -1.0, 1.0)

        var input = com.arcrobotics.ftclib.geometry.Vector2d(strafeSpeed, forwardSpeed)
        input = input.rotateBy(-gyroAngle)

        val theta = input.angle()

        var wheelSpeeds = DoubleArray(4)
        wheelSpeeds[0] = Math.sin(theta + Math.PI / 4)
        wheelSpeeds[3] = Math.sin(theta - Math.PI / 4)
        wheelSpeeds[1] = Math.sin(theta - Math.PI / 4)
        wheelSpeeds[2] = Math.sin(theta + Math.PI / 4)

        val newList = normalize(wheelSpeeds, input.magnitude())

        newList[0] += turnSpeed
        newList[3] -= turnSpeed
        newList[1] += turnSpeed
        newList[2] -= turnSpeed

        val secondNewList = normalize(newList)

        setMotorPowers(secondNewList[0], secondNewList[1], secondNewList[2], secondNewList[3])
        robot.t.addData("leftFront", secondNewList[0])
        robot.t.update()
//        update()
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

}