package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.geometry.Vector2d
import org.firstinspires.ftc.teamcode.subsystems.Turret
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.sign

class TurretLock45(
    private val turret: Turret,
    private val heading: () -> Double,
    val input: () -> Vector2d
) : CommandBase() {
    override fun initialize() {
        addRequirements(turret)
    }

    override fun execute() {
        val vec = input.invoke()
        val head = heading.invoke()
        val ang = atan2(vec.y,vec.x) * 180.0/ PI
        var target = ((head * 180.0/ PI) - ang)
        var factor = 0.0
        if (sign(head) != 0.0) factor = sign(head)
        target *= factor

        turret.targetAngle = target
    }

    override fun isFinished() = false
}