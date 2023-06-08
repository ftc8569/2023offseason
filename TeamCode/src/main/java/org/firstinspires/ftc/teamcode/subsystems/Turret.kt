package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utilities.Constants.MOTOR_TO_TURRET_GEAR_RATIO
import org.firstinspires.ftc.teamcode.utilities.Constants.TURRET_KA
import org.firstinspires.ftc.teamcode.utilities.Constants.TURRET_KD
import org.firstinspires.ftc.teamcode.utilities.Constants.TURRET_KI
import org.firstinspires.ftc.teamcode.utilities.Constants.TURRET_KP
import org.firstinspires.ftc.teamcode.utilities.Constants.TURRET_KV
import org.firstinspires.ftc.teamcode.utilities.Constants.TURRET_MAX_A
import org.firstinspires.ftc.teamcode.utilities.Constants.TURRET_MAX_V
import org.firstinspires.ftc.teamcode.utilities.Constants.TURRET_MOTOR_TICKS_PER_REV

/*
* TODO:
** Deal with nullable stuff (utilize type safety)
* */

class Turret(val motor: MotorEx) : SubsystemBase() {
    init {
        motor.setRunMode(Motor.RunMode.VelocityControl)
    }

    var curAngle = 0.0
    var targetAngle = 0.0
    var curPosition = 0
    var targetPosition = 0
    var atTarget = true
    private var profile: MotionProfile? = null
    private var time: ElapsedTime = ElapsedTime()
    var profiled = true
    var controller = PIDController(TURRET_KP, TURRET_KI, TURRET_KD)

    fun setTargetPosition(targetPosition: Int) {
        this.targetPosition = targetPosition
        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(curPosition.toDouble(), motor.velocity, motor.acceleration),
            MotionState(targetPosition.toDouble(), 0.0, 0.0),
            TURRET_MAX_V,
            TURRET_MAX_A
        )
        time.reset()
    }

    fun setTargetAngle(targetAngle: Double) {
        this.targetAngle = targetAngle
        setTargetPosition(angleToEncoderTicks(targetAngle).toInt())
    }

    override fun periodic() {
        atTarget = kotlin.math.abs(curPosition - targetPosition) < 10
        if (!atTarget && profiled) {
            val curX = motor.currentPosition
            val tempXTarget = profile?.get(time.time())?.x
            val tempVTarget = profile?.get(time.time())?.v
            val tempATarget = profile?.get(time.time())?.a
            var output = controller.calculate(
                curX.toDouble(),
                tempXTarget!!.toDouble(),
            ) + TURRET_KV * tempVTarget!! + TURRET_KA * tempATarget!!
            motor.set(output)
        } else motor.set(0.0)
    }

    // Angles in degrees
    private fun angleToEncoderTicks(angle: Double): Double {
        return (angle / 360) * (TURRET_MOTOR_TICKS_PER_REV / MOTOR_TO_TURRET_GEAR_RATIO)
    }

    init {
        register()
    }

}