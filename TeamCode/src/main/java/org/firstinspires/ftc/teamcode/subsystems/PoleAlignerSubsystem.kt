package org.firstinspires.ftc.teamcode.subsystems

import androidx.core.math.MathUtils.clamp
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.utilities.ScoringConfigs.ALIGNER_HOME

class PoleAlignerSubsystem(private val robot: Robot, val servo: ServoImplEx) : SubsystemBase() {
    val maxAngleRange = 300.0 //degree
    val pwmRange = PwmControl.PwmRange(500.0,2500.0)
    val homeAngle = convertPositionToAngle(ALIGNER_HOME)

    init {
        servo.pwmRange = pwmRange
        register()
    }
    var angle : Double = homeAngle

    public fun home() {
        angle = homeAngle
    }
    override fun periodic() {
        servo.position = convertAngleToPosition(angle)
    }
    fun convertAngleToPosition(angle: Double) : Double {
        return clamp((angle / (maxAngleRange / 2.0)) + 0.5, 0.0, 1.0)
    }
    fun convertPositionToAngle(position: Double) : Double {
        return (clamp(position, 0.0, 1.0) - 0.5) * maxAngleRange
    }
}