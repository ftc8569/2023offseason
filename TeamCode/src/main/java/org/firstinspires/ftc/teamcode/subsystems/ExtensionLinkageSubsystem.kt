package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.Cons.*

// Angle naming conventions
// phi -- angle between slides and leg A (shorter linkage leg)
// theta -- angle between the two linkage legs

class ExtensionLinkageSubsystem(val robot: Robot, val servo: ServoImplEx) : SubsystemBase() {
    init {
        servo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        servo.position = EXTENSION_HOME
        register()
    }

    val isExenteded : Boolean
        get() = servo.position > EXTENSION_HOME

    var position : Double
        get() = servo.position
        set(value) {
            servo.position = value
        }

    fun home(){
        servo.position = EXTENSION_HOME
    }
}