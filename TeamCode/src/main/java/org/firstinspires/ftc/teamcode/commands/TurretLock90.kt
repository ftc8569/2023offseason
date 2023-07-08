package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.geometry.Vector2d
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.utilities.HelperFunctions
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2

class TurretLock90(
    private val turret: Turret,
    private val heading: () -> Double,
    private val input: () -> Vector2d
) : CommandBase() {
    init {
        addRequirements(turret)
    }
    private val potentialAngles = listOf<Double>(0.0, -PI / 2, PI, PI / 2)
    override fun execute() {
        val vec = input.invoke()
        val robotHeading = HelperFunctions.normalizeAngleRadians(heading.invoke())
        val joystickAngle = HelperFunctions.normalizeAngleRadians(atan2(vec.x, vec.y))

        var closestAngle = potentialAngles[0]
        var closestDistance = 10000.0
        for (i in potentialAngles.indices) {
            val distToAngle = abs(potentialAngles[i] - joystickAngle)
            if (distToAngle < closestDistance) {
                closestAngle = potentialAngles[i]
                closestDistance = distToAngle
            }
        }
        if(vec.x == 0.0 && vec.y ==0.0){
            closestAngle = PI;
        }
        val robotRelativeAngle = HelperFunctions.toRobotRelativeAngle(
            closestAngle,
            robotHeading
        )

        val angleDegrees = HelperFunctions.toDegrees(robotRelativeAngle)

        turret.targetAngle = angleDegrees

        print(turret)
    }
    override fun isFinished() = false
}