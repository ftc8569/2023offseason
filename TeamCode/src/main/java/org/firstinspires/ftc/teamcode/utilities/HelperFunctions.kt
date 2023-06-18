package org.firstinspires.ftc.teamcode.utilities

object HelperFunctions {
    fun getAngleFromEncoders(
        current: Double,
        verticalCount: Double,
        horizontalCount: Double
    ): Double {
        return (current - verticalCount) / (verticalCount - horizontalCount) * 90 + 90
    }
}