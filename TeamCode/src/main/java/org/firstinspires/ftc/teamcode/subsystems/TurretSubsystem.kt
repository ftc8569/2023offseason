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
import org.firstinspires.ftc.teamcode.utilities.PostAutoPoses.*
import kotlin.math.abs

// Put timer in constructor, mock a timer
class TurretSubsystem(val robot: Robot, val motor: MotorEx) : SubsystemBase() {

    // TODO this TURRET_ANGLE needs to be changed to the actual starting angle of the turret for left and right
    var angleStartOffsetDegrees = TURRET_ANGLE
    var currentAngleDegrees = 0.0
    var numWraps = 0
    var targetAngleDegrees = 0.0
    var pid = BasicPID(PIDCoefficients(TURRET_KP, TURRET_KI, TURRET_KD))
    var controller = AngleController(pid)
    var previous_reference = 0.0;
    var m_profile = MotionProfileGenerator.generateMotionProfile(
        MotionState(0.0,0.0,0.0),
        MotionState(0.0,0.0,0.0),
        {Math.toRadians(180.0)},
        {Math.toRadians(90.0)})
    var timer = ElapsedTime()
    var isTelemetryEnabled = false
    val closeEnoughToTargetAngleDegrees = 1.0
    init {
        motor.setRunMode(Motor.RunMode.RawPower)

        // if there are any saved encoder angles from the end of autonomous, use them
        if(RelativeEncoderStateMonitorSubsystem.savedEncoderAngles != null) {
            angleStartOffsetDegrees = RelativeEncoderStateMonitorSubsystem.savedEncoderAngles!!.turretAngle
        }
        else {
            angleStartOffsetDegrees = TURRET_STARTING_ANGLE
            motor.resetEncoder()
        }
        register()
    }

    override fun periodic() {
        currentAngleDegrees = encodersToAngle(motor.currentPosition.toDouble()) + angleStartOffsetDegrees
        numWraps = (abs(currentAngleDegrees) / 360.0).toInt()

        val currentAngleRadians = Math.toRadians(currentAngleDegrees)
        val targetAngleRadians = Math.toRadians(targetAngleDegrees)

        // Raw PID for unit testing, can add motion profile back in later
        generateMotionProfile(targetAngleRadians, currentAngleRadians)
        var motion_profile_reference = m_profile[timer.seconds()].x

        val pidPower = clamp(controller.calculate(motion_profile_reference, currentAngleRadians), -1.0, 1.0)
        motor.set(pidPower)

        if (isTelemetryEnabled) {
            robot.telemetry.addLine("Turret Telemetr")
            robot.telemetry.addData("Target Angle", targetAngleDegrees)
            robot.telemetry.addData("Current Angle", currentAngleDegrees)
            robot.telemetry.update()
        }
    }

    // Angles in degrees
    private fun angleToEncoderTicks(angle: Double): Double {
        return (angle / 360) * (TURRET_MOTOR_TICKS_PER_REV / MOTOR_TO_TURRET_GEAR_RATIO)
    }
    private fun encodersToAngle(ticks: Double): Double {
        return (ticks * 360 * MOTOR_TO_TURRET_GEAR_RATIO) / TURRET_MOTOR_TICKS_PER_REV
    }
    private fun generateMotionProfile(reference: Double, state: Double) {
        if (previous_reference == reference) {
            return
        }
        previous_reference = reference
        m_profile = MotionProfileGenerator.generateMotionProfile(
            MotionState(state, 0.0, 0.0),
            MotionState(reference, 0.0, 0.0),
            { Math.toRadians(360.0) },
            { Math.toRadians(900.0) },
        )
        timer.reset()
    }

    public fun isCloseEnoughToTargetAngle(tolerance : Double = closeEnoughToTargetAngleDegrees) : Boolean {
        return abs(currentAngleDegrees - targetAngleDegrees) < closeEnoughToTargetAngleDegrees
    }


}