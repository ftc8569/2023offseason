package org.firstinspires.ftc.teamcode.subsystems

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.util.epsilonEquals
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Cons.MOTOR_TO_TURRET_GEAR_RATIO
import org.firstinspires.ftc.teamcode.Cons.TURRET_KD
import org.firstinspires.ftc.teamcode.Cons.TURRET_KI
import org.firstinspires.ftc.teamcode.Cons.TURRET_KP
import org.firstinspires.ftc.teamcode.Cons.TURRET_MAX_ANGULAR_ACCELERATION
import org.firstinspires.ftc.teamcode.Cons.TURRET_MAX_ANGULAR_VELOCITY
import org.firstinspires.ftc.teamcode.Cons.TURRET_MOTOR_TICKS_PER_REV
import kotlin.math.abs

// Put motionProfileTimer in constructor, mock a motionProfileTimer
class TurretSubsystem   (val robot: Robot, val motor: MotorEx, private val homingResult: HomingResult) : SubsystemBase() {

    private val imu = robot.hardwareMap.get<IMU>(IMU::class.java, "imu2") // imu on the expansion hub (turret imu)

    init {
        val parameters = IMU.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN))
        imu.initialize(parameters)

        motor.setRunMode(Motor.RunMode.RawPower)

        // if we are homing with the limit switch, reset the encoder
        if(homingResult.method == HomingMethod.LIMIT_SWITCH) {
            motor.resetEncoder()
        }

        register()
    }

    val angleStartOffset = homingResult.homeAngles.turretAngle
    var currentAngle = if (homingResult.method == HomingMethod.USING_SAVED_FROM_AUTO)
                            getCurrentAngleFromEncoder()
                        else
                            homingResult.homeAngles.turretAngle
        private set

    val angleRange = AngleRange(-360.0, 360.0)
    var targetAngle = currentAngle
        set(value) {
            field = value.coerceIn(angleRange.minimumAngle, angleRange.maximumAngle)
        }

    val pid = BasicPID(PIDCoefficients(TURRET_KP, TURRET_KI, TURRET_KD))

    var maxAngularVelocity = TURRET_MAX_ANGULAR_VELOCITY
    var maxAngularAcceleration = TURRET_MAX_ANGULAR_ACCELERATION

    var previousTarget = targetAngle;
    var motionProfile = MotionProfileGenerator.generateMotionProfile(
        MotionState(currentAngle,.0,0.0,0.0),
        MotionState(targetAngle,0.0,0.0),
        { maxAngularVelocity },
        { maxAngularAcceleration })
    val motionProfileTimer = ElapsedTime()
    var isTelemetryEnabled = false
    var useMotionProfile = true
    var isEnabled = true

    val closeEnoughToTargetAngleDegrees = 1.0

    fun getCurrentAngleFromEncoder() : Double {
        return encodersToAngleDegrees(motor.currentPosition.toDouble()) + angleStartOffset
    }
    var deltaTimer = ElapsedTime()
    private var angularX : Double = currentAngle
    private var angularV : Double = 0.0
    private var angularA : Double = 0.0

    override fun periodic() {
        currentAngle = encodersToAngleDegrees(motor.currentPosition.toDouble()) + angleStartOffset

        val deltaT = deltaTimer.seconds()
        deltaTimer.reset()

        val newAngularV = (currentAngle - angularX) / deltaT
        val newAngularA = (newAngularV - angularV) / deltaT
        angularX = currentAngle
        angularV = newAngularV
        angularA = newAngularA

        val pidPower =
            if(useMotionProfile) {

                // using the current motion profile target as the starting point for the next motion profile
                // this is to ensure continuity between motion profiles (eliminates jitter where the motor as moved past current angle
                // and the motion profile will generate a first location that is backwards from the current direction of motion)
                val currentMotionProfileX = motionProfile[motionProfileTimer.seconds()].x
                generateMotionProfile(targetAngle, currentMotionProfileX, angularV, angularA)
                val intermediateTargetAngle = motionProfile[motionProfileTimer.seconds()].x
                pid.calculate( intermediateTargetAngle, currentAngle).coerceIn(-1.0, 1.0)
            } else {
                pid.calculate(targetAngle, currentAngle).coerceIn(-1.0, 1.0)
            }
        if(isEnabled)
            motor.set(pidPower)
        else
            motor.set(0.0)

        if (isTelemetryEnabled) {
            robot.telemetry.addLine("TurretTelemetry")
            robot.telemetry.addData("IsEnabled", isEnabled)
            robot.telemetry.addData("TargetAngle", targetAngle)
            robot.telemetry.addData("CurrentAngle", currentAngle)
            robot.telemetry.addData("AngularV", angularV)
            robot.telemetry.addData("AngularA", angularA)

            val imuHeading = angleStartOffset - imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
            robot.telemetry.addData("imu heading", imuHeading)
            robot.telemetry.update()
        }
    }
    private fun angleDegreesToEncoderTicks(angle: Double): Double {
        return (angle / 360) * (TURRET_MOTOR_TICKS_PER_REV / MOTOR_TO_TURRET_GEAR_RATIO)
    }
    private fun encodersToAngleDegrees(ticks: Double): Double {
        return (ticks * 360 * MOTOR_TO_TURRET_GEAR_RATIO) / TURRET_MOTOR_TICKS_PER_REV
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
    public fun isCloseEnoughToTargetAngle(tolerance : Double = closeEnoughToTargetAngleDegrees) : Boolean {
        return abs(currentAngle - targetAngle) < closeEnoughToTargetAngleDegrees || motor.velocity epsilonEquals 0.0
    }

}