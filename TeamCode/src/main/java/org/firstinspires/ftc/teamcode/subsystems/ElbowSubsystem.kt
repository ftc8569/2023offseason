package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.teamcode.Cons.*
import org.firstinspires.ftc.teamcode.utilities.PostAutoPoses
import kotlin.math.abs

class ElbowSubsystem(val robot: Robot, private val motor1: MotorEx, private val motor2: MotorEx) :
    SubsystemBase() {
    private val controller = PIDController(ELBOW_KP, ELBOW_KI, ELBOW_KD)
    private val motors = MotorGroup(motor1, motor2)
    private val encoderTicksPerRevolution = 2782
    private val closeEnoughToTargetAngleDegrees = 1.0

    var isTelemetryEnabled = false
    var targetAngleDegrees = 0.0
    var isEnabled = true
    var angleStartOffset = ELBOW_START_ANGLE
    val currentAngleDegrees : Double
        get() = getCurrentElbowAngle()

    init {
        register()
        motor1.inverted = true
        motors.setRunMode(Motor.RunMode.RawPower)
        motors.resetEncoder()

        // TODO make this better. This is a hacky way to do it
        // We're basically assuming that if all the values are in the static class are equal to zero
        // then they probably haven't been edited. The best guess that we can make about our position
        // is that we're in our starting position
        if (PostAutoPoses.TURRET_ANGLE == 0.0 && PostAutoPoses.ELBOW_ANGLE == 0.0 && PostAutoPoses.DRIVETRAIN_HEADING == 0.0) {
            angleStartOffset = ELBOW_START_ANGLE
        }
    }

    override fun periodic() {
        var power = 0.0

        if (isEnabled) {
            val currentAngle = getCurrentElbowAngle()
            power = controller.calculate(currentAngle, targetAngleDegrees)

            // TODO: ensure that this the right way. I think it is, but I'm not sure
            if (robot.extension.isExenteded) power += 0.1
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
    public fun isCloseEnoughToTargetAngle(tolerance : Double = closeEnoughToTargetAngleDegrees) : Boolean {
        return abs(currentAngleDegrees - targetAngleDegrees) < closeEnoughToTargetAngleDegrees
    }
    private fun getCurrentElbowAngle(): Double {
        // just use the first motor encoder
        return convertEncoderTicksToDegrees(motors.positions[0])  + angleStartOffset
    }
    private fun convertEncoderTicksToDegrees(ticks: Double): Double {
        return (ticks / encoderTicksPerRevolution) * 360.0
    }
}