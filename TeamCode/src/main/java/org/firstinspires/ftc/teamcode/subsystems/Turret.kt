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
import org.firstinspires.ftc.teamcode.utilities.PostAutoPoses.*
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.sign

// Put timer in constructor, mock a timer
class Turret(val motor: MotorEx, val robot: Robot, val headingSupplier: ()-> Double) : SubsystemBase() {
    init {
        motor.setRunMode(Motor.RunMode.RawPower)
    }

    var angleStartOffset = 0.0
    init {
        // We're basically assuming that if all the values are in the static class are equal to zero
        // then they probably haven't been edited. The best guess that we can make about our position
        // is that we're in our starting position
        angleStartOffset = if(TURRET_ANGLE == 0.0 && ELBOW_ANGLE == 0.0 && DRIVETRAIN_HEADING == 0.0 ){
            TURRET_STARTING_ANGLE
        } else {
            TURRET_ANGLE
        }
    }
    var calcOutput = 0.0
    var numWraps = 0
    var fieldRelativeControl = false

    var curAngle = 0.0
    var fieldRelativeTargetAngle = 0.0
    var targetAngle = 0.0
        set(targetAngle: Double) {
            field = targetAngle
            this.targetPosition = angleToEncoderTicks(targetAngle).toInt() - angleToEncoderTicks(angleStartOffset).toInt()
        }
    var targetPosition = 0
    var atTarget = false
    var controller = PIDController(TURRET_KP, TURRET_KI, TURRET_KD)

    override fun periodic() {
        if (fieldRelativeControl) {
            targetAngle = if(numWraps > 2){
                -HelperFunctions.toDegrees(
                    HelperFunctions.toRobotRelativeAngle(
                        HelperFunctions.toRadians(fieldRelativeTargetAngle),
                        HelperFunctions.toRadians(headingSupplier.invoke())
                    )
                )
            } else {
                -HelperFunctions.toDegrees(
                    HelperFunctions.toRobotRelativeAngleNoNorm(
                        HelperFunctions.toRadians(fieldRelativeTargetAngle),
                        HelperFunctions.toRadians(headingSupplier.invoke())
                    )
                )
            }
        } else {
            targetAngle = fieldRelativeTargetAngle
        }

        val curX = motor.currentPosition

        curAngle = encodersToAngle(curX.toDouble()) + angleStartOffset
        atTarget = kotlin.math.abs(targetAngle - curAngle) < 5
        numWraps = (abs(curAngle) / 360.0).toInt()

        // Raw PID for unit testing, can add motion profile back in later
        calcOutput =
            clamp(controller.calculate(curX.toDouble(), targetPosition.toDouble()), -1.0, 1.0)

        motor.set(calcOutput)

        robot.t.addData("Robot relative target angle", targetAngle)
        robot.t.addData("Current Angle", curAngle)
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
        register()
    }


}