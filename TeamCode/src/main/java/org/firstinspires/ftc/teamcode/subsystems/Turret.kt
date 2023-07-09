package org.firstinspires.ftc.teamcode.subsystems

import androidx.core.math.MathUtils.clamp
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Cons.*
import org.firstinspires.ftc.teamcode.utilities.HelperFunctions
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.sign

// Put timer in constructor, mock a timer
class Turret(val motor: MotorEx, val robot: Robot) : SubsystemBase() {
    init {
        motor.setRunMode(Motor.RunMode.RawPower)
    }

    var calcOutput = 0.0
    var adjustedOutput = 0.0
    var numWraps = 0

    var curAngle = 0.0
    var fieldRelativeTargetAngle = 0.0
    var targetAngle = 0.0
        set(targetAngle: Double) {
            field = targetAngle
            this.targetPosition = angleToEncoderTicks(targetAngle).toInt()
        }
    var curPosition = 0
    var targetPosition = 0
    var atTarget = true
    var profiled = true
    var controller = PIDController(TURRET_KP, TURRET_KI, TURRET_KD)

    override fun periodic() {
        atTarget = kotlin.math.abs(curPosition - targetPosition) < 10
        targetAngle = if(numWraps > 2){
            -HelperFunctions.toDegrees(
                HelperFunctions.toRobotRelativeAngle(
                    HelperFunctions.toRadians(fieldRelativeTargetAngle),
                    HelperFunctions.toRadians(robot.drivetrain.getYaw())
                )
            )
        } else {
            -HelperFunctions.toDegrees(
                HelperFunctions.toRobotRelativeAngleNoNorm(
                    HelperFunctions.toRadians(fieldRelativeTargetAngle),
                    HelperFunctions.toRadians(robot.drivetrain.getYaw())
                )
            )
        }

        val curX = motor.currentPosition
        curAngle = encodersToAngle(curX.toDouble())
        numWraps = (abs(curAngle) / 360.0).toInt()

        if (!atTarget && profiled) {

            // Raw PID for unit testing, can add motion profile back in later
            calcOutput =
                clamp(controller.calculate(curX.toDouble(), targetPosition.toDouble()), -1.0, 1.0)

            motor.set(calcOutput)

        } else motor.set(0.0)

        robot.t.addData("Field Relative Target Angle", fieldRelativeTargetAngle)
        robot.t.addData("Robot relative target angle", targetAngle)
        robot.t.addData("Current Angle", curAngle)
        robot.t.addData("Num wraps", numWraps)
        robot.t.update()
    }

    // Angles in degrees
    private fun angleToEncoderTicks(angle: Double): Double {
        return (angle / 360) * (TURRET_MOTOR_TICKS_PER_REV / MOTOR_TO_TURRET_GEAR_RATIO)
    }

    private fun encodersToAngle(ticks: Double): Double {
        return (ticks * 360 * MOTOR_TO_TURRET_GEAR_RATIO) / TURRET_MOTOR_TICKS_PER_REV
    }

    init {
        controller.setTolerance(13.0)
        register()
    }


//                var velocity = motor.velocity
//            var vContribution = clamp(velocity * TURRET_KV, -1.0, 1.0)
//            val plannedAccel = (calcOutput - vContribution) / TURRET_KA
//            var accel =
//                sign(plannedAccel) * Math.min(abs(plannedAccel), abs(TURRET_MAX_A)) * TURRET_KA
//            adjustedOutput = vContribution + accel
}