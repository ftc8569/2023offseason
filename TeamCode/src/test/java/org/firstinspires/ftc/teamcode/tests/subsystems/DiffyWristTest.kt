package org.firstinspires.ftc.teamcode.tests.subsystems

import junit.framework.TestCase.assertEquals
import org.firstinspires.ftc.teamcode.subsystems.DiffWrist
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import org.junit.Test
import org.mockito.kotlin.doAnswer
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.mock


class DiffyWristTest {

    @Test
    fun axonCRServoTest(){
        var loopCount = 0
        val servo = mock<AxonCRServo>(){
            on {position} doAnswer {
                loopCount++
                loopCount * 360.0
            }
        }
        for (i in 0..8){
            servo.position
        }
        assertEquals(3600.0, servo.position)
    }

    @Test
    fun differentialOdometryTest(){
        var loopCount = 0
        val robot = mock<Robot>()
        val leftServo = mock<AxonCRServo>(){
            on {position} doAnswer {
                loopCount++
                loopCount * 360.0
            }
        }
        val rightServo = mock<AxonCRServo>(){
            on {position} doAnswer {
                loopCount++
                loopCount * 360.0
            }
        }
        val wrist = DiffWrist(leftServo, rightServo, robot)
        wrist.periodic()
        println(wrist.translation)
        println(wrist.rotation)
    }

}