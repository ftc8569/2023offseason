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

    var position = 1.0
        set(pos){
            servo.position = pos
            field = pos
        }
    var extended = false

    init {
        servo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }

    fun home(){
        servo.position = 1.0
        extended = false
    }


    init {
        register()
    }
}