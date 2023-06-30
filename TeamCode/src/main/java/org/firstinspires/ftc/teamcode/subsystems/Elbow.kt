package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.teamcode.utilities.Constants.ELBOW_KA
import org.firstinspires.ftc.teamcode.utilities.Constants.ELBOW_KCOS
import org.firstinspires.ftc.teamcode.utilities.Constants.ELBOW_KD
import org.firstinspires.ftc.teamcode.utilities.Constants.ELBOW_KI
import org.firstinspires.ftc.teamcode.utilities.Constants.ELBOW_KP
import org.firstinspires.ftc.teamcode.utilities.Constants.ELBOW_KS
import org.firstinspires.ftc.teamcode.utilities.Constants.ELBOW_KV
import org.firstinspires.ftc.teamcode.utilities.Constants.INITIAL_ANGLE
import org.firstinspires.ftc.teamcode.utilities.HelperFunctions
import kotlin.math.PI
import kotlin.math.cos

class Elbow(private val motor1: MotorEx, private val motor2: MotorEx) :
    SubsystemBase() {
    private val controller = PIDController(ELBOW_KP, ELBOW_KI, ELBOW_KD)
    var targetAngle = 0.0
    var currentAngle = 0.0
    var enabled = true
    private val ff = 0.5 // TUNE THIS
    init {
        motor2.inverted = true
    }
    val motor = MotorGroup(motor1, motor2)
    private fun getFeedforward(extensionDistance: Double, angle: Double) =
        ff * extensionDistance * cos(angle * (180 / PI))

    override fun periodic() {
        if (enabled) {
            currentAngle =
                (motor1.currentPosition / 696.5) * 90
//            val power = controller.calculate(currentAngle, targetAngle) + getFeedforward(robot.extension.length, targetAngle)
//            motor.set(power)
        }
    }
}