package org.firstinspires.ftc.teamcode.commands.turret

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import org.apache.commons.math3.util.FastMath.round
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem

class ControlTurretAngle(val turret : TurretSubsystem, val snapAngle: Double,  val turrentAngleProvider : () -> Vector2d) : CommandBase() {
    init {
        addRequirements(turret)
        turret.isTelemetryEnabled = true
    }
    override fun execute() {
        val vector = turrentAngleProvider.invoke()
        val magnitude = vector.norm()
        if (magnitude > 0.9) {
            val joystickAngle =  normalizeDegrees(Math.toDegrees(vector.angle()))
            val currentAngle = turret.currentAngleDegrees
            val angleDiff = shortestArc(currentAngle, joystickAngle)
            val newAngle = currentAngle + angleDiff
            turret.targetAngleDegrees = roundToNearestSnap(newAngle, snapAngle)
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