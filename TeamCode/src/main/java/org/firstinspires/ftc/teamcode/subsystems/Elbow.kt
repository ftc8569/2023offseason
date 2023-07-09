package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.teamcode.Cons.*
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos

class Elbow(private val motor1: MotorEx, private val motor2: MotorEx, val robot:Robot) :
    SubsystemBase() {
    private val controller = PIDController(ELBOW_KP, ELBOW_KI, ELBOW_KD)
    var targetAngle = 0.0
    var currentAngle = 0.0
    var enabled = true
    var atTargetPosition = true
    init {
        motor1.inverted = true
        motor1.setRunMode(Motor.RunMode.RawPower)
        motor2.setRunMode(Motor.RunMode.RawPower)
    }
    val motor = MotorGroup(motor1, motor2)
    init {
        motor.resetEncoder()
    }


    override fun periodic() {
        if (enabled) {
            currentAngle =
                (motor.positions[0] / 696.5) * 90 + ELBOW_START_ANGLE

            atTargetPosition = abs(targetAngle - currentAngle) < 10

            var power = controller.calculate(currentAngle, targetAngle)
            if(robot.extension.extended) power += 0.1
            motor.set(power)

        }
    }

    init {
        register()
    }
}