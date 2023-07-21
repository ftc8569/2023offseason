package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.util.epsilonEquals
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Cons.*
import kotlin.math.abs
import kotlin.math.cos

class ElbowSubsystem(private val robot: Robot, motor1: MotorEx, motor2: MotorEx, private val homingResult: HomingResult) : SubsystemBase() {
    private val motors = MotorGroup(motor1, motor2)
    var isEnabled = true
    var isTelemetryEnabled = false

    private val controller = PIDController(ELBOW_KP, ELBOW_KI, ELBOW_KD)
    private val encoderTicksPerRevolution = 2782
    private val closeEnoughToTargetAngleDegrees = 1.0

    private val maxAngularVelocity = ELBOW_MAX_ANGULAR_VELOCITY
    private val maxAngularAcceleration = ELBOW_MAX_ANGULAR_ACCELERATION
    private val angleStartOffsetDegrees = homingResult.homeAngles.elbowAngle
    val angleRange = AngleRange(-45.0, 58.0)
    var currentAngleDegrees : Double = homingResult.homeAngles.elbowAngle
        private set
    var targetAngle = currentAngleDegrees
        set(value) {
            field = value.coerceIn(angleRange.minimumAngle, angleRange.maximumAngle)
        }
    val motionProfileTimer = ElapsedTime()
    var previousTarget = targetAngle
    var motionProfile = MotionProfileGenerator.generateMotionProfile(
        MotionState(currentAngleDegrees, 0.0, 0.0),
        MotionState(targetAngle, 0.0, 0.0),
        { maxAngularVelocity },
        { maxAngularAcceleration },
    )
    init {
        register()
        motor1.inverted = true
        motors.setRunMode(Motor.RunMode.RawPower)

        // if we are homing with the limit switch, reset the encoder
        if(homingResult.method == HomingMethod.LIMIT_SWITCH) {
            motors.resetEncoder()
        }
        currentAngleDegrees = getCurrentElbowAngle()
    }


    var deltaTimer = ElapsedTime()

    private var angularX : Double = getCurrentElbowAngle()
    private var angularV : Double = 0.0
    private var angularA : Double = 0.0

    override fun periodic() {
        var power = 0.0
        val deltaT = deltaTimer.seconds()
        deltaTimer.reset()

        currentAngleDegrees = getCurrentElbowAngle()
        val newAngularV = (currentAngleDegrees - angularX) / deltaT
        val newAngularA = (newAngularV - angularV) / deltaT
        angularX = currentAngleDegrees
        angularV = newAngularV
        angularA = newAngularA

        // using the current motion profile target as the starting point for the next motion profile
        // this is to ensure continuity between motion profiles (eliminates jitter where the motor as moved past current angle
        // and the motion profile will generate a first location that is backwards from the current direction of motion)
        val currentMotionProfileX = motionProfile[motionProfileTimer.seconds()].x
        generateMotionProfile(targetAngle, currentMotionProfileX, angularV, angularA)

        if (isEnabled) {
            power = controller.calculate(currentAngleDegrees, motionProfile[motionProfileTimer.seconds()].x)

            val gravityFeedForward = ELBOW_GRAVITY_FEED_FORWARD_COEFFICIENT *
                    robot.extension.targetLength/ExtensionLinkageSubsystem.MAXIMUM_EXTENSION  *
                    cos(Math.toRadians(currentAngleDegrees))

            power += gravityFeedForward
        }

        motors.set(power)

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Elbow: Telemetry Enabled")
            robot.telemetry.addData("IsEnabled:", isEnabled)
            robot.telemetry.addData("Target Angle:", targetAngle)
            robot.telemetry.addData("Current Angle:", currentAngleDegrees)
            robot.telemetry.addData("Angular V:", angularV)
            robot.telemetry.addData("Angular A:", angularA)
            robot.telemetry.addData("Power:", power)
            robot.telemetry.update()
        }
    }
    fun isCloseEnoughToTargetAngle(tolerance : Double = closeEnoughToTargetAngleDegrees) : Boolean {
        return abs(currentAngleDegrees - targetAngle) < closeEnoughToTargetAngleDegrees || motors.speeds[0] epsilonEquals 0.0
    }

    private fun generateMotionProfile(target: Double, currentX: Double, currentV: Double, currentA: Double) {
        if (!(previousTarget epsilonEquals target)) {
            previousTarget = target
            motionProfile = MotionProfileGenerator.generateMotionProfile(
                MotionState(currentX, currentV, currentA),
                MotionState(target, 0.0, 0.0),
                { maxAngularVelocity },
                { maxAngularAcceleration },
            )
            motionProfileTimer.reset()
        }
    }
    private fun getCurrentElbowAngle(): Double {
        // just use the first motor encoder
        return convertEncoderTicksToDegrees(motors.positions[0])  + angleStartOffsetDegrees
    }
    private fun convertEncoderTicksToDegrees(ticks: Double): Double {
        return (ticks / encoderTicksPerRevolution) * 360.0
    }
}