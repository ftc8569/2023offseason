package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.util.ElapsedTime
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.sign

class AxonCRServo(
    hw: HardwareMap,
    servoName: String?,
    analogName: String?,
    min: Double,
    max: Double
) {
    private val servo: CRServoImplEx
    private val analogInput: AnalogInput
    private val min: Double
    private val max: Double
    private val loopTimer = ElapsedTime()
    private var lastLoopTime = 0.0
    var velocity = 0.0 // in degrees per second
        private set
    private val analogOutput: Double
        get() { return analogInput.voltage / 3.3 * 360}
    private var lastAngleReading = 0.0
    var position = 0.0

    init {
        servo = hw.get(CRServoImplEx::class.java, servoName)
        analogInput = hw.get(AnalogInput::class.java, analogName)
        servo.pwmRange = PwmRange(min, max)
        this.min = min
        this.max = max
        lastAngleReading = analogOutput
        loopTimer.reset()
    }

    fun setPower(power: Double) {
        servo.power = power
    }

    // Must be called on every loop
    fun update():Unit{
        lastLoopTime = loopTimer.time(TimeUnit.SECONDS).toDouble()
        val prevAngle = lastAngleReading
        val currentAngle = analogOutput
        var angleDelta = currentAngle - prevAngle

        // TODO: Check value of 180 degrees
        // Accounts for potentiometer wrap around
        if(abs(angleDelta) > 180.0){
            angleDelta += 360.0 * -sign(angleDelta)
        }
        velocity = angleDelta / lastLoopTime
        position += angleDelta

        loopTimer.reset()
    }
}