package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.geometry.Vector2d
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.utilities.HelperFunctions
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2

class TurretLock45(
    private val turret: Turret,
    private val heading: () -> Double,
    private val input: () -> Vector2d
) : CommandBase() {
    override fun initialize() {
        addRequirements(turret)
    }
    private val potentialAngles = listOf<Double>(-3 * PI / 4, -PI / 4, PI / 4, 3 * PI / 4)
    override fun execute() {
        val vec = input.invoke()
        val robotHeading = HelperFunctions.normalizeAngleRadians(heading.invoke())
        val joystickAngle = HelperFunctions.normalizeAngleRadians(atan2(vec.y, vec.x))

        var closestAngle = potentialAngles[0]
        var closestDistance = 10000.0
        for (i in potentialAngles.indices) {
            val distToAngle = abs(potentialAngles[i] - joystickAngle)
            if (distToAngle < closestDistance) {
                closestAngle = potentialAngles[i]
                closestDistance = distToAngle
            }
        }
        turret.targetAngle = HelperFunctions.toDegrees(
            HelperFunctions.toRobotRelativeAngle(
                closestAngle,
                robotHeading
            )
        )
    }
    override fun isFinished() = false
}