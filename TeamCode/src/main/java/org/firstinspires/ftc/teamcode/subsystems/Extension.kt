package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import org.firstinspires.ftc.teamcode.Cons.*
import org.firstinspires.ftc.teamcode.utilities.HelperFunctions
import kotlin.math.*

// Angle naming conventions
// phi -- angle between slides and leg A (shorter linkage leg)
// theta -- angle between the two linkage legs

class Extension(val servo: AxonCRServo) : SubsystemBase() {
    private val controller = PIDController(EXTENSION_KP, EXTENSION_KI, 0.0)
    private val phiOffset = 16.3

    var phi = phiOffset
    var targetPhi = phi
    var theta = 0.0
    var targetTheta = theta // Called targetTheta, but we're not really controlling theta, we are controlling phi
    var initialPosition = servo.position
    var targetPosition = initialPosition
    var power = 0.0

    var length = 0.0
        set(len) {
            // Solve law of cosines for length
            val boundedLen = max(len, sqrt(LEG_A.pow(2) + LEG_B.pow(2) - 2 * LEG_A * LEG_B))
            targetTheta = acos((LEG_A.pow(2) + LEG_B.pow(2) - boundedLen.pow(2)) / (2 * LEG_A * LEG_B))
            targetPhi =
                180 - asin((LEG_B * sin(targetTheta)) / boundedLen) * (180 / Math.PI)
            this.targetPosition = 2 * targetPhi + (initialPosition - 2 * phiOffset)
            field = boundedLen
        }

    init {
        servo.servo.direction = DcMotorSimple.Direction.REVERSE
    }

    fun home(){
        this.targetPosition = initialPosition
    }

    override fun periodic() {
        // Update state variables
        phi = (servo.position - initialPosition) / 2 + phiOffset
        theta = 180 - phi - asin((LEG_A * sin(PI - HelperFunctions.toRadians(phi)))/ LEG_B)

        servo.update()
        controller.setPID(sin(HelperFunctions.toRadians(theta)) * EXTENSION_KP, 0.0, 0.0)
        val out = controller.calculate(servo.position, targetPosition)
        power = out
        servo.setPower(out)
    }

    init {
        register()
    }
}