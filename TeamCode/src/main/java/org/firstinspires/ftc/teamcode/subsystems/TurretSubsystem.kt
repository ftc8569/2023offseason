package org.firstinspires.ftc.teamcode.subsystems

import androidx.core.math.MathUtils.clamp
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Cons.*
import org.opencv.core.Mat
import kotlin.math.abs

// Put timer in constructor, mock a timer
class TurretSubsystem(val robot: Robot, val motor: MotorEx, private val homingResult: HomingResult) : SubsystemBase() {

    val angleStartOffsetDegrees = homingResult.homeAngles.turretAngle
    var currentAngleDegrees = homingResult.homeAngles.turretAngle
        private set

    val angleRange = AngleRange(-360.0, 360.0)
    var targetAngleDegrees = 0.0
        set(value) {
            field = value.coerceIn(angleRange.minimumAngle, angleRange.maximumAngle)
        }

    val pid = BasicPID(PIDCoefficients(TURRET_KP, TURRET_KI, TURRET_KD))
    val controller = AngleController(pid)

    var maxAngularVelocity = TURRET_MAX_ANGULAR_VELOCITY
    var maxAngularAcceleration = TURRET_MAX_ANGULAR_ACCELERATION
    var previousTarget = 0.0;
    var motionProfile = MotionProfileGenerator.generateMotionProfile(
        MotionState(Math.toRadians(currentAngleDegrees),.0,0.0,0.0),
        MotionState(Math.toRadians(targetAngleDegrees),0.0,0.0),
        { Math.toRadians(maxAngularVelocity) },
        { Math.toRadians(maxAngularAcceleration) })
    val motionProfileTimer = ElapsedTime()


    var isTelemetryEnabled = false
    var useMotionProfile = true
    var isEnabled = true

    val closeEnoughToTargetAngleDegrees = 1.0
    init {
        motor.setRunMode(Motor.RunMode.RawPower)

        // if we are homing with the limit switch, reset the encoder
        if(homingResult.method == HomingMethod.LIMIT_SWITCH) {
            motor.resetEncoder()
        }

        currentAngleDegrees = encodersToAngleDegrees(motor.currentPosition.toDouble()) + angleStartOffsetDegrees
        register()
    }

    override fun periodic() {
        currentAngleDegrees = encodersToAngleDegrees(motor.currentPosition.toDouble()) + angleStartOffsetDegrees

        val currentAngleRadians = Math.toRadians(currentAngleDegrees)
        val targetAngleRadians = Math.toRadians(targetAngleDegrees)

        val pidPower =
            if(useMotionProfile) {
                generateMotionProfileInAngleRadians(targetAngleRadians, currentAngleRadians)
                val intermediateTargetAngleRadians = motionProfile[motionProfileTimer.seconds()].x
                clamp(controller.calculate(intermediateTargetAngleRadians, currentAngleRadians), -1.0, 1.0)
            } else {
                clamp(controller.calculate(targetAngleRadians, currentAngleRadians), -1.0, 1.0)
            }

        if(isEnabled)
            motor.set(pidPower)
        else
            motor.set(0.0)

        if (isTelemetryEnabled) {
            robot.telemetry.addLine("TurretTelemetry")
            robot.telemetry.addData("IsEnabled", isEnabled)
            robot.telemetry.addData("TargetAngle", targetAngleDegrees)
            robot.telemetry.addData("CurrentAngle", currentAngleDegrees)
            robot.telemetry.update()
        }
    }
    private fun angleDegreesToEncoderTicks(angle: Double): Double {
        return (angle / 360) * (TURRET_MOTOR_TICKS_PER_REV / MOTOR_TO_TURRET_GEAR_RATIO)
    }
    private fun encodersToAngleDegrees(ticks: Double): Double {
        return (ticks * 360 * MOTOR_TO_TURRET_GEAR_RATIO) / TURRET_MOTOR_TICKS_PER_REV
    }
    private fun generateMotionProfileInAngleRadians(target: Double, current: Double) {
        if (!nearlyEqual(previousTarget, target)) {
            previousTarget = target
            motionProfile = MotionProfileGenerator.generateMotionProfile(
                MotionState(current, 0.0, 0.0),
                MotionState(target, 0.0, 0.0),
                { Math.toRadians(maxAngularVelocity) },
                { Math.toRadians(maxAngularAcceleration) },
            )
            motionProfileTimer.reset()
            robot.telemetry.addData("Gen Motion Prof Target = ", target )
            robot.telemetry.update()
        }
    }
    public fun isCloseEnoughToTargetAngle(tolerance : Double = closeEnoughToTargetAngleDegrees) : Boolean {
        return abs(currentAngleDegrees - targetAngleDegrees) < closeEnoughToTargetAngleDegrees
    }
    private fun nearlyEqual(a: Double, b: Double, epsilon: Double = 1E-10): Boolean {
        return abs(a - b) < epsilon
    }

}