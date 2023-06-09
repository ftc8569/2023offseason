package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.ServoImplEx

class AxonServo(
    hw: HardwareMap,
    servoName: String,
    analogName: String,
    min: Double,
    max: Double
) {
    private val servo: ServoImplEx
    private val analogInput: AnalogInput
    private val min: Double
    private val max: Double
    var target = 0.0

    init {
        servo = hw.get(ServoImplEx::class.java, servoName)
        analogInput = hw.get(AnalogInput::class.java, analogName)
        servo.pwmRange = PwmRange(min, max)
        this.min = min
        this.max = max
    }

    fun setPosition(pos: Double) {
        target = (pos - min) / (max - min)
        servo.position = target
    }

    val position: Double
        get() = analogInput.voltage / 3.3 * 360
}