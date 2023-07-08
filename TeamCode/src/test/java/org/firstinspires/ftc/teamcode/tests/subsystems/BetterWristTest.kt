package org.firstinspires.ftc.teamcode.tests.subsystems

import org.junit.Test

class BetterWristTest {

    @Test
    fun wrist_test(){
        val bend = 0.0
        val twist = 90.0
        var leftAng = 0.5 + ((bend/90)*1000 - ((twist/90 * 1000)/2))/2500
        var rightAng = 0.5 + ((bend/90)*1000 + ((twist/90 * 1000)/2))/2500

        println(leftAng)
        println(rightAng)
    }


}