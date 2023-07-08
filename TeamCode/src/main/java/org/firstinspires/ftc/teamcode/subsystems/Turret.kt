package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Cons.MOTOR_TO_TURRET_GEAR_RATIO
import org.firstinspires.ftc.teamcode.Cons.TURRET_KD
import org.firstinspires.ftc.teamcode.Cons.TURRET_KI
import org.firstinspires.ftc.teamcode.Cons.TURRET_KP
import org.firstinspires.ftc.teamcode.Cons.TURRET_MAX_A
import org.firstinspires.ftc.teamcode.Cons.TURRET_MAX_V
import org.firstinspires.ftc.teamcode.Cons.TURRET_MOTOR_TICKS_PER_REV

// Put timer in constructor, mock a timer
class Turret(val motor: MotorEx, val robot: Robot) : SubsystemBase() {
    init {
        motor.setRunMode(Motor.RunMode.VelocityControl)
    }

    var curAngle = 0.0
    var targetAngle = 0.0
        set(targetAngle: Double){
            field = targetAngle
            this.targetPosition = angleToEncoderTicks(targetAngle).toInt()
        }
    var curPosition = 0
    var targetPosition = 0
        set(targetPosition: Int){
            field = targetPosition
            this.profile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(curPosition.toDouble(), motor.velocity, motor.acceleration),
                MotionState(targetPosition.toDouble(), 0.0, 0.0),
                TURRET_MAX_V,
                TURRET_MAX_A
            )
            time.reset()
        }
    var atTarget = true
    private lateinit var profile: MotionProfile
    private var time: ElapsedTime = ElapsedTime()
    var profiled = true
    var controller = PIDController(TURRET_KP, TURRET_KI, TURRET_KD)

    override fun periodic() {
        atTarget = kotlin.math.abs(curPosition - targetPosition) < 10
        if (!atTarget && profiled) {
            val curX = motor.currentPosition
            val tempXTarget = profile[time.time()].x
            val tempVTarget = profile[time.time()].v
            val tempATarget = profile[time.time()].a
//            var output = TURRET_KV * tempVTarget + TURRET_KA * tempATarget

            // Raw PID for unit testing, can add motion profile back in later
            var output = controller.calculate(curX.toDouble(), targetPosition.toDouble())
            output.clamp(-0.5, 0.5)
            motor.set(output)

            robot.t.addData("Turret targ", targetAngle)
        } else motor.set(0.0)
    }

    // Angles in degrees
    private fun angleToEncoderTicks(angle: Double): Double {
        return (angle / 360) * (TURRET_MOTOR_TICKS_PER_REV / MOTOR_TO_TURRET_GEAR_RATIO)
    }

//    fun encoderTicksToAngle(ticks: Double): Double {
//
//    }
    init {
        controller.setTolerance(13.0)
        register()
    }

}