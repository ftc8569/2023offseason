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

class Extension(private val servo: AxonCRServo, val t: Telemetry) : SubsystemBase() {
    val dashboard = FtcDashboard.getInstance()
    val mulT = MultipleTelemetry(dashboard.telemetry, t)
    val controller = PIDController(EXTENSION_KP, EXTENSION_KI, EXTENSION_KD)
    var targetPosition = 0.0
    var length = 0.0
        set(len) {
            // Solve law of cosines for length
            val inside = (LEG_A.pow(2) + LEG_B.pow(2) - len.pow(2))/(2 * LEG_A * LEG_B)
            val thetaPrime = acos(inside)
            val theta = 180-asin((LEG_B * sin(thetaPrime))/len) * (180/Math.PI)
            this.targetPosition = 5 * theta
            field = len
        }
    init {
        servo.servo.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun periodic(){
        servo.update()
        val power = controller.calculate(servo.position, servo.target)
        mulT.addData("servo position", servo.position)
        mulT.addData("servo power", power)
        mulT.addData("analog out", servo.analogOutput)
        mulT.addData("goal", targetPosition)
        mulT.update()
        servo.setPower(power)
    }

    init {
        register()
    }
}