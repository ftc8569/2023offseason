package org.firstinspires.ftc.teamcode.commands.turret

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import org.apache.commons.math3.util.FastMath.round
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem

class ControlTurretAngle(private val turret : TurretSubsystem,
                         private val snapAngle: Double = 45.0,
                         private val turrentAngleProvider : () -> Vector2d) : CommandBase() {
    init {
        addRequirements(turret)
    }
    private val minimumMagnitude = 0.9

    fun setTurretAngle(angle: Double) {
        turret.targetAngle = angle
    }
    override fun execute() {
        val vector = turrentAngleProvider.invoke()
        val magnitude = vector.norm()
        if (magnitude > minimumMagnitude) {
            val joystickAngle =  normalizeDegrees(Math.toDegrees(vector.angle()))
            val currentAngle = turret.currentAngle
            val angleDiff = shortestArc(currentAngle, joystickAngle)
            val newAngle = currentAngle + angleDiff
            turret.targetAngle = roundToNearestSnap(newAngle, snapAngle)
        }
    }
    override fun isFinished(): Boolean {
        return false
    }
    fun roundToNearestSnap(angle: Double, snapAngle: Double): Double {
        return round(angle / snapAngle) * snapAngle
    }
    fun normalizeDegrees(angle: Double): Double {
        var normalizedAngle = angle % 360.0
        if (normalizedAngle > 180.0) {
            normalizedAngle -= 360.0
        } else if (normalizedAngle < -180.0) {
            normalizedAngle += 360.0
        }
        return normalizedAngle
    }
    fun shortestArc(current: Double, target: Double): Double {
        var diff = normalizeDegrees(target - current)

        if (diff > 180.0) {
            diff -= 360.0
        } else if (diff < -180.0) {
            diff += 360.0
        }

        return diff
    }
}