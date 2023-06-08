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

class Elbow(private val motor1: MotorEx, private val motor2: MotorEx) : SubsystemBase() {
    val motor = MotorGroup(motor1, motor2)
    val ff = ArmFeedforward(
        ELBOW_KS,
        ELBOW_KCOS,
        ELBOW_KV,
        ELBOW_KA
    )
    val controller = PIDController(ELBOW_KP, ELBOW_KI, ELBOW_KD)
    var targetAngle = INITIAL_ANGLE
    var currentAngle = INITIAL_ANGLE
    var enabled = true

    override fun periodic(){
        if(enabled){
            currentAngle = motor1.currentPosition.toDouble() // TODO: CONVERT TO ANGLE
//            var output = ff.
        }
    }

}