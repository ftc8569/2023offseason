package org.firstinspires.ftc.teamcode.tests.commands

import com.arcrobotics.ftclib.geometry.Vector2d
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import junit.framework.TestCase.assertEquals
import org.firstinspires.ftc.teamcode.commands.TurretLock45
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.junit.Test
import org.mockito.kotlin.mock
import kotlin.math.PI

class TurretLock45Tests {

    @Test
    fun locks_to_45(){
        val turret = Turret(mock<MotorEx>())
        val heading = {0.0}
        val input = {Vector2d(1.0, 0.5)}
        val command = TurretLock45(turret, heading, input)
        command.execute()
        assertEquals(45.0,turret.targetAngle, 1e-5)
    }

    @Test
    fun locks_to_neg_45(){
        val turret = Turret(mock<MotorEx>())
        val heading = {0.0}
        val input = {Vector2d(-1.0, 0.5)}
        val command = TurretLock45(turret, heading, input)
        command.execute()
        assertEquals(-45.0,turret.targetAngle, 1e-5)
    }

    @Test
    fun locks_to_135(){
        val turret = Turret(mock<MotorEx>())
        val heading = {0.0}
        val input = {Vector2d(1.0, -0.5)}
        val command = TurretLock45(turret, heading, input)
        command.execute()
        assertEquals(135.0,turret.targetAngle, 1e-5)
    }

    @Test
    fun locks_to_neg_135(){
        val turret = Turret(mock<MotorEx>())
        val heading = {0.0}
        val input = {Vector2d(-1.0, -0.5)}
        val command = TurretLock45(turret, heading, input)
        command.execute()
        assertEquals(-135.0,turret.targetAngle, 1e-5)
    }

}