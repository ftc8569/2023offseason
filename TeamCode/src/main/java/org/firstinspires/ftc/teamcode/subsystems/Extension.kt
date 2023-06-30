package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import org.firstinspires.ftc.teamcode.utilities.Constants.*
import kotlin.math.*

class Extension(val servo: AxonCRServo) : SubsystemBase() {
    private val controller = PIDController(EXTENSION_KP, EXTENSION_KI, EXTENSION_KD)
    private val thetaOffset = 16.3
    var theta = thetaOffset
    var targetPosition = servo.position
    var power = 0.0

    var inside = 0.0
    var thetaPrime = 0.0


    var length = 0.0
        set(len) {
            // Solve law of cosines for length
            val boundedLen = min(len, sqrt(LEG_A.pow(2) + LEG_B.pow(2) - 2 * LEG_A * LEG_B))
            inside = (LEG_A.pow(2) + LEG_B.pow(2) - boundedLen.pow(2)) / (2 * LEG_A * LEG_B)
            thetaPrime = acos(inside)
            theta =
                180 - asin((LEG_B * sin(thetaPrime)) / boundedLen) * (180 / Math.PI) + thetaOffset
            this.targetPosition = 2 * theta
            field = boundedLen
        }

    init {
        servo.servo.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun periodic() {
        servo.update()
        val out = controller.calculate(servo.position, targetPosition)
        power = out
        servo.setPower(out)
    }

    init {
        register()
    }
}