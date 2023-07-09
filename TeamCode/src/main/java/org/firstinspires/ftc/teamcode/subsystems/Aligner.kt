package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.utilities.AxonServo
import org.firstinspires.ftc.teamcode.utilities.ScoringConfigs.ALIGNER_HOME

class Aligner(val servo: ServoImplEx, private val robot: Robot) : SubsystemBase() {

    var position = ALIGNER_HOME
        set(pos) {
            servo.position = pos
            field = pos
        }

    init {
        servo.pwmRange = PwmControl.PwmRange(500.0,2500.0)
        position = ALIGNER_HOME
    }

    init {
        register()
    }
}