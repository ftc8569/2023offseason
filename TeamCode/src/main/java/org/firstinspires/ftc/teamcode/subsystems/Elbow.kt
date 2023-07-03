package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.teamcode.Cons.ELBOW_KD
import org.firstinspires.ftc.teamcode.Cons.ELBOW_KI
import org.firstinspires.ftc.teamcode.Cons.ELBOW_KP
import kotlin.math.PI
import kotlin.math.cos

class Elbow(private val motor1: MotorEx, private val motor2: MotorEx, private val robot: Robot) :
    SubsystemBase() {
    private val controller = PIDController(ELBOW_KP, ELBOW_KI, ELBOW_KD)
    var targetAngle = 0.0
    var currentAngle = 0.0
    var enabled = true
    private val ff = 0.5 // TUNE THIS
    init {
        motor1.inverted = true
        motor1.setRunMode(Motor.RunMode.RawPower)
        motor2.setRunMode(Motor.RunMode.RawPower)
    }
    val motor = MotorGroup(motor1, motor2)
    init {
        motor.resetEncoder()
    }
    private fun getFeedforward(extensionDistance: Double, angle: Double) =
        ff * extensionDistance * cos(angle * (180 / PI))

    override fun periodic() {
        if (enabled) {
            currentAngle =
                (motor.positions[0] / 696.5) * 90
//            val power = controller.calculate(currentAngle, targetAngle) + getFeedforward(robot.extension.length, targetAngle)

            var power = controller.calculate(currentAngle, targetAngle)
            if(robot.extension.extended) power += 0.1
            motor.set(power)
        }
    }

    init {
        register()
    }
}