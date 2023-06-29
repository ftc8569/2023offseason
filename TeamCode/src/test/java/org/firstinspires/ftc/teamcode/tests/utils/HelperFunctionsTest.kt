package org.firstinspires.ftc.teamcode.tests.utils

import junit.framework.TestCase.assertEquals
import org.firstinspires.ftc.teamcode.utilities.HelperFunctions
import org.junit.Test
import kotlin.math.PI

class HelperFunctionsTest {

    @Test
    fun angles_normalize_properly(){
        var angle = 2 * PI
        var norm_angle = HelperFunctions.normalizeAngleRadians(angle)
        assertEquals(0.0, norm_angle, 0.0001)

        angle = PI
        norm_angle = HelperFunctions.normalizeAngleRadians(angle)
        assertEquals(PI, norm_angle, 0.0001)

        angle = -2 * PI
        norm_angle = HelperFunctions.normalizeAngleRadians(angle)
        assertEquals(0.0, norm_angle, 0.0001)

        angle = (-5 * PI) / 4
        norm_angle = HelperFunctions.normalizeAngleRadians(angle)
        assertEquals((3* PI/4), norm_angle, 0.0001)
    }

    // Field relative to absolute angles
    @Test
    fun robot_zero_heading(){
        var turretFieldAngle = PI/4
        var robotHeading = 0.0
        var absoluteAngle = HelperFunctions.toRobotRelativeAngle(turretFieldAngle, robotHeading)
        assertEquals(PI/4, absoluteAngle, 1E-5)
    }
    @Test
    fun robot_90_degrees(){
        var turretFieldAngle = PI/4
        var robotHeading = PI/2
        var absoluteAngle = HelperFunctions.toRobotRelativeAngle(turretFieldAngle, robotHeading)
        assertEquals(-PI/4, absoluteAngle, 1E-5)
    }
    @Test
    fun robot_neg_90_degrees(){
        var turretFieldAngle = PI/4
        var robotHeading = -PI/2
        var absoluteAngle = HelperFunctions.toRobotRelativeAngle(turretFieldAngle, robotHeading)
        assertEquals(3*PI/4, absoluteAngle, 1E-5)
    }
    @Test
    fun robot_90_turret_neg_45(){
        var turretFieldAngle = -PI/4
        var robotHeading = PI/2
        var absoluteAngle = HelperFunctions.toRobotRelativeAngle(turretFieldAngle, robotHeading)
        assertEquals(-3*PI/4, absoluteAngle, 1E-5)
    }
    @Test
    fun robot_neg_90_turret_neg_45(){
        var turretFieldAngle = -PI/4
        var robotHeading = -PI/2
        var absoluteAngle = HelperFunctions.toRobotRelativeAngle(turretFieldAngle, robotHeading)
        assertEquals(PI/4, absoluteAngle, 1E-5)
    }
}