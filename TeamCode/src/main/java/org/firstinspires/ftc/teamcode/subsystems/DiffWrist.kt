package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo

class DiffWrist(
    private val leftServo: AxonCRServo,
    private val rightServo: AxonCRServo,
    val robot: Robot
) : SubsystemBase() {
    private val rightController = PIDController(0.0, 0.0, 0.0)
    private val leftController = PIDController(0.0, 0.0, 0.0)

    init {
        leftServo.reversed = true
    }

    private val leftInitial = leftServo.position
    private val rightInitial = rightServo.position

    var leftTarget = leftInitial
        set(target) {
            field = target + leftInitial
        }
    var rightTarget = rightInitial
        set(target) {
            field = target + rightInitial
        }

    fun setTargets(left:Double, right:Double){
        leftTarget = left
        rightTarget = right
    }

    override fun periodic() {
        /* Each servo will calculate its loop time by storing the degrees travelled
        *  since the last time update() was called */
        leftServo.update()
        rightServo.update()

        var leftPower = leftController.calculate(leftServo.position - leftInitial, leftTarget)
        var rightPower = leftController.calculate(rightServo.position - rightInitial, rightTarget)

        leftServo.setPower(leftPower)
        rightServo.setPower(rightPower)
    }
}