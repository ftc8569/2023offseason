package org.firstinspires.ftc.teamcode.subsystems

import androidx.core.math.MathUtils.clamp
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

class PoleAlignerSubsystem(private val robot: Robot, val servo: ServoImplEx) : SubsystemBase() {
    val maxAngleRange = 300.0 //degree
    val pwmRange = PwmControl.PwmRange(500.0,2500.0)
    val homeAngle = ArmStatePositionData.ARM_HOME.aligner.angle
    val servoSpeedEstimate = 0.09 //seconds per 60 degrees @ 6V (goBilda Speed Servo)
    val timer = ElapsedTime()
    private var estimatedTimeToComplete = 0.0 //seconds
    val angleRange = AngleRange(-149.0, 30.0)
    val maximumMovementTime = abs(angleRange.maximumAngle - angleRange.minimumAngle) * servoSpeedEstimate / 60.0 //seconds

    init {
        servo.pwmRange = pwmRange
        register()
    }
    var angle : Double = homeAngle
        set(value) {
            val previousAngle = field
            val newAngle = clamp(value, angleRange.minimumAngle, angleRange.maximumAngle)
            val angleChange = kotlin.math.abs(newAngle - previousAngle)
            estimatedTimeToComplete = (angleChange * servoSpeedEstimate / 60.0).coerceAtMost(maximumMovementTime)
            timer.reset()
            field = newAngle
        }

    public fun home() {
        angle = homeAngle
    }
    fun movementShouldBeComplete() : Boolean {
        return timer.seconds() > estimatedTimeToComplete
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