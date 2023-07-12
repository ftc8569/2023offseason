package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.teamcode.Cons.*
import kotlin.math.abs

class ElbowSubsystem(private val robot: Robot, motor1: MotorEx, motor2: MotorEx, private val homingResult: HomingResult) : SubsystemBase() {
    private val controller = PIDController(ELBOW_KP, ELBOW_KI, ELBOW_KD)
    private val motors = MotorGroup(motor1, motor2)
    private val encoderTicksPerRevolution = 2782
    private val closeEnoughToTargetAngleDegrees = 1.0

    var isTelemetryEnabled = false
    private val angleStartOffsetDegrees = homingResult.homeAngles.elbowAngle
    val angleRange = AngleRange(-50.0, 58.0)
    var targetAngleDegrees = 0.0
        set(value) {
            field = value.coerceIn(angleRange.minimumAngle, angleRange.maximumAngle)
        }
    var isEnabled = true
    val currentAngleDegrees : Double
        get() = getCurrentElbowAngle()

    init {
        register()
        motor1.inverted = true
        motors.setRunMode(Motor.RunMode.RawPower)

        // if we are homing with the limit switch, reset the encoder
        if(homingResult.method == HomingMethod.LIMIT_SWITCH) {
            motors.resetEncoder()
        }

    }

    override fun periodic() {
        var power = 0.0

        if (isEnabled) {
            val currentAngle = getCurrentElbowAngle()
            power = controller.calculate(currentAngle, targetAngleDegrees)

            // TODO: ensure that this the right way. I think it is, but I'm not sure
            if (robot.extension.isExtended) power += 0.1
        }

        motors.set(power)

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Elbow: Telemetry Enabled")
            robot.telemetry.addData("IsEnabled:", isEnabled)
            robot.telemetry.addData("Target Angle:", targetAngleDegrees)
            robot.telemetry.addData("Current Angle:", getCurrentElbowAngle())
            robot.telemetry.addData("Power:", power)
            robot.telemetry.update()
        }
    }
    fun isCloseEnoughToTargetAngle(tolerance : Double = closeEnoughToTargetAngleDegrees) : Boolean {
        return abs(currentAngleDegrees - targetAngleDegrees) < closeEnoughToTargetAngleDegrees
    }
    private fun getCurrentElbowAngle(): Double {
        // just use the first motor encoder
        return convertEncoderTicksToDegrees(motors.positions[0])  + angleStartOffsetDegrees
    }
    private fun convertEncoderTicksToDegrees(ticks: Double): Double {
        return (ticks / encoderTicksPerRevolution) * 360.0
    }
}