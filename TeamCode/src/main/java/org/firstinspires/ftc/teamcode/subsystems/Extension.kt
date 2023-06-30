package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import org.firstinspires.ftc.teamcode.utilities.AxonServo
import org.firstinspires.ftc.teamcode.utilities.Constants.*
import kotlin.math.acos
import kotlin.math.asin
import kotlin.math.pow
import kotlin.math.sin

class Extension(private val servo: AxonCRServo) : SubsystemBase() {
    private val controller = PIDController(EXTENSION_KP, EXTENSION_KI, EXTENSION_KD)
    private val thetaOffset = 16.3
    var theta = thetaOffset
    var targetPosition = 0.0
    var power = 0.0
    var length = 0.0
        set(len) {
            // Solve law of cosines for length
            val inside = (LEG_A.pow(2) + LEG_B.pow(2) - len.pow(2))/(2 * LEG_A * LEG_B)
            val thetaPrime = acos(inside)
            theta = 180-asin((LEG_B * sin(thetaPrime))/len) * (180/Math.PI) + thetaOffset
            this.targetPosition = 2 * theta
            field = len
        }
    init {
        servo.servo.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun periodic(){
        servo.update()
        power = controller.calculate(servo.position, targetPosition)
        servo.setPower(power)
    }

    init {
        register()
    }
}