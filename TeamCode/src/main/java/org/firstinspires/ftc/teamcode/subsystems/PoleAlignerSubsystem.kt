package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.utilities.ScoringConfigs.ALIGNER_HOME

class PoleAlignerSubsystem(private val robot: Robot, val servo: ServoImplEx) : SubsystemBase() {
    init {
        servo.pwmRange = PwmControl.PwmRange(500.0,2500.0)
        servo.position = ALIGNER_HOME
        register()
    }

    var position : Double
        get() = servo.position
        set(value) {
            servo.position = value
        }
}