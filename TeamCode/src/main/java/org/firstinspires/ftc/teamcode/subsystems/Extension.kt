package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import org.firstinspires.ftc.teamcode.Cons.*
import org.firstinspires.ftc.teamcode.utilities.HelperFunctions
import kotlin.math.*

// Angle naming conventions
// phi -- angle between slides and leg A (shorter linkage leg)
// theta -- angle between the two linkage legs

class Extension(val servo: ServoImplEx) : SubsystemBase() {

    var position = EXTENSION_HOME
        set(pos){
            var temp = pos
//            if(temp > EXTENSION_MAX){
//                temp = EXTENSION_MAX
//            }
//            if (temp < EXTENSION_HOME){
//                temp = EXTENSION_HOME
//            }
            servo.position = temp

            field = temp
        }
    var extended = false

    init {
        servo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }

    fun home(){
        servo.position = EXTENSION_HOME
        extended = false
    }


    init {
        register()
    }
}