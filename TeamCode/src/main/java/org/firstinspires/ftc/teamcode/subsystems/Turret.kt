package org.firstinspires.ftc.teamcode.subsystems

import androidx.core.math.MathUtils.clamp
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.acmerobotics.roadrunner.profile.AccelerationConstraint
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.profile.VelocityConstraint
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Cons.*
import org.firstinspires.ftc.teamcode.utilities.HelperFunctions
import org.firstinspires.ftc.teamcode.utilities.PostAutoPoses.*
import kotlin.math.abs

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
    var previous_reference = 0.0;

    var m_profile = MotionProfileGenerator.generateMotionProfile(
                                                                MotionState(0.0,0.0,0.0),
                                                                MotionState(0.0,0.0,0.0),
                                                                {Math.toRadians(180.0)},
                                                                {Math.toRadians(90.0)})

    var timer = ElapsedTime()

    var curAngle = 0.0

    // if fieldRelativeControl == true, this is a fieldRelative angle, and a transform will be applied to get the robot relative angle
    // otherwise, this will be the same as [robotRelativeTargetAngle]
    var targetAngle = 0.0

    // This will always be the absolute angle of the turret
    var robotRelativeTargetAngle = 0.0
        set(targetAngle: Double) {
            field = targetAngle
            this.targetPosition = angleToEncoderTicks(targetAngle).toInt() - angleToEncoderTicks(angleStartOffset).toInt()
        }

    var targetPosition = 0
    var atTarget = false

    var pid = BasicPID(PIDCoefficients(TURRET_KP, TURRET_KI, TURRET_KD))
    var controller = AngleController(pid)

    override fun periodic() {
        if (fieldRelativeControl) {
            robotRelativeTargetAngle = if(numWraps > 2){
                -HelperFunctions.toDegrees(
                    HelperFunctions.toRobotRelativeAngle(
                        HelperFunctions.toRadians(targetAngle),
                        HelperFunctions.toRadians(headingSupplier.invoke())
                    )
                )
            } else {
                -HelperFunctions.toDegrees(
                    HelperFunctions.toRobotRelativeAngleNoNorm(
                        HelperFunctions.toRadians(targetAngle),
                        HelperFunctions.toRadians(headingSupplier.invoke())
                    )
                )
            }
        } else {
            robotRelativeTargetAngle = targetAngle
        }

        val curX = motor.currentPosition

        curAngle = encodersToAngle(curX.toDouble()) + angleStartOffset
        atTarget = abs(robotRelativeTargetAngle - curAngle) < 5
        numWraps = (abs(curAngle) / 360.0).toInt()
        var curAngleRad = Math.toRadians(curAngle)

        // Raw PID for unit testing, can add motion profile back in later
        var reference = Math.toRadians(encodersToAngle(targetPosition.toDouble()))
        var state = curAngleRad

        generateMotionProfile(reference, state)

        var motion_profile_reference = m_profile[timer.seconds()].x

        calcOutput =
            clamp(controller.calculate(motion_profile_reference, state), -1.0, 1.0)

        motor.set(calcOutput)

        robot.t.addData("Robot relative target angle", robotRelativeTargetAngle)
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


}