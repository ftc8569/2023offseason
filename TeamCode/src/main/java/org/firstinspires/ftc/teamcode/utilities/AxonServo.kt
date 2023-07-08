package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.ServoImplEx

class AxonServo(
    hw: HardwareMap,
    servoName: String,
    min: Double,
    max: Double
) {
    val servo: ServoImplEx
    private val min: Double
    private val max: Double

    init {
        servo = hw.get(ServoImplEx::class.java, servoName)
        servo.pwmRange = PwmRange(min, max)
        this.min = min
        this.max = max
    }
}