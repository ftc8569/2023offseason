package org.firstinspires.ftc.teamcode.tests

import com.arcrobotics.ftclib.hardware.motors.MotorEx
import junit.framework.TestCase.assertEquals
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.junit.Test
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.mock

class TurretTest {
    @Test
    fun threeSixtyReturns4416(){
        val motor = mock<MotorEx>()
        val turret = Turret(motor)

        assertEquals(4416.0, turret.angleToEncoderTicks(360.0),1.0)
    }
}
