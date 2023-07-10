package org.firstinspires.ftc.teamcode.utilities

import androidx.core.math.MathUtils.clamp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.ServoImplEx

class AxonServo(
    hw: HardwareMap,
    servoName: String,
    minimumPulseWidth: Double,
    maximumPulseWidth: Double,
    private val maximumRotationRangeDegrees :Double = 180.0)
{
    val servo: ServoImplEx

    init {
        servo = hw.get(ServoImplEx::class.java, servoName)
        servo.pwmRange = PwmRange(minimumPulseWidth, maximumPulseWidth)
    }

    fun getServoPositionFromAngleDegrees(angle : Double) : Double {
        val servoMiddlePosition = 0.5
        return clamp(angle/maximumRotationRangeDegrees + servoMiddlePosition, 0.0, 1.0)
    }
    fun getServoPulseWidthFromAngleDegrees(angle : Double) : Double {
        val servoPosition = getServoPositionFromAngleDegrees(angle)
        return getServoPulseWidthFromPosition(servoPosition)
    }

    fun getServoPulseWidthFromPosition(position : Double) : Double {
        return position * (servo.pwmRange.usPulseUpper - servo.pwmRange.usPulseLower) + servo.pwmRange.usPulseLower
    }
}