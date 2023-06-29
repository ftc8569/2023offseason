package org.firstinspires.ftc.teamcode.utilities

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

object HelperFunctions {
    fun getAngleFromEncoders(
        current: Double,
        verticalCount: Double,
        horizontalCount: Double
    ): Double {
        return (current - verticalCount) / (verticalCount - horizontalCount) * 90 + 90
    }

    /**
     * @param fieldRelativeAngle the desired angle of the mechanism relative to the driver [-PI to PI]
     * @param robotHeading the heading of the robot [-PI to PI]
     * @return the robot relative angle to achieve desired driver relative angle [-PI to PI]
     */
    fun toRobotRelativeAngle(fieldRelativeAngle: Double, robotHeading: Double): Double {
        val normFieldRelAngle = normalizeAngleRadians(fieldRelativeAngle)
        val normRobotHeading = normalizeAngleRadians(robotHeading)
        return normalizeAngleRadians(normFieldRelAngle + abs(normRobotHeading) * -sign(normRobotHeading))
    }

    fun normalizeAngleRadians(angle: Double): Double {
        var newAngle = angle
        if (abs(newAngle) > PI) newAngle -= (2 * PI) * sign(newAngle)
        return newAngle % (2 * PI)
    }

    fun toRadians(angle: Double) = angle * (PI/180)
    fun toDegrees(angle: Double) = angle * (180/PI)
}