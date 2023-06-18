package org.firstinspires.ftc.teamcode.tests.utils

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServoImplEx
import junit.framework.TestCase.assertEquals
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import org.junit.Test
import org.mockito.kotlin.doAnswer
import org.mockito.kotlin.mock

class AxonCRServoTest {

    @Test
    fun test_update_fun(){
        var numEncoderCalls = 0
        val servo = mock<CRServoImplEx>(){
            on {power} doAnswer{
                var out = 0.0
                if(numEncoderCalls % 2 ==0) out = 1.0
                if(numEncoderCalls % 2 ==1) out = -1.0
                out
            }
        }
        val encoder = mock<AnalogInput>(){
            on{voltage} doAnswer {
                var out = 0.0
                when(numEncoderCalls){
                    0 -> out = 0.0
                    1 -> out = 10 * (3.3 / 360)
                    2 -> out = 350 * (3.3/360)
                    3 -> out = 20 * (3.3/360)
                    4 -> out = 40 * (3.3/360)
                    5 -> out = 200 * (3.3/360)
                    6 -> out = 300 * (3.3/360)
                    7 -> out = 10 * (3.3/360)
                }
                numEncoderCalls++
                out
            }
        }

        val crservo = AxonCRServo(servo, encoder, 500.0, 2500.0)
        crservo.update()
        assertEquals(10.0, crservo.position, 0.1)
        crservo.update()
        assertEquals(-10.0, crservo.position, 0.1)
        crservo.update()
        assertEquals(20.0, crservo.position, 0.1)
        crservo.update()
        assertEquals(40.0, crservo.position, 0.1)
        crservo.update()
        assertEquals(200.0, crservo.position, 0.1)
        crservo.update()
        assertEquals(300.0, crservo.position, 0.1)
        crservo.update()
        assertEquals(370.0, crservo.position, 0.1)
    }
}