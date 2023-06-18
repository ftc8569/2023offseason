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
                    1 -> out = 200 * (3.3 / 360)
                    2 -> out = 10 * (3.3/360)
                }
                numEncoderCalls++
                out
            }
        }

        val crservo = AxonCRServo(servo, encoder, 500.0, 2500.0)
        assertEquals(0.0, crservo.offset)
        crservo.update()
        assertEquals(-200.0, crservo.lastPosition, 1.0)
        crservo.update()


    }
}