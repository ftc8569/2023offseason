package org.firstinspires.ftc.teamcode.subsystems

import androidx.core.math.MathUtils.clamp
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.util.epsilonEquals
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Cons.*
import org.firstinspires.ftc.teamcode.utilities.AxonServo
import kotlin.math.*

// Angle naming conventions
// phi -- angle between slides and leg A (shorter linkage leg)
// theta -- angle between the two linkage legs

class ExtensionLinkageSubsystem(val robot: Robot, val servo: AxonServo) : SubsystemBase() {
    init {
        register()
    }

    companion object {
        const val MINIMUM_EXTENSION = 0.25
        const val MAXIMUM_EXTENSION = 13.0
        const val GROUND = 5.19
        const val LOW = 8.0
        const val MID = 10.0
        const val HIGH = MAXIMUM_EXTENSION
        const val PICKUP = MINIMUM_EXTENSION
    }
    val analogOutput = robot.hardwareMap.get(AnalogInput::class.java, "extension")
    var isTelemetryEnabled = false

    var targetLength : Double = MINIMUM_EXTENSION
        set(value) {
            field = clamp(value, MINIMUM_EXTENSION, HIGH)
            servo.angle = getServoAngleFromExtensionLength(targetLength)
        }
    private fun getServoAngleFromExtensionLength(extensionLength: Double) : Double {
        return baseLinkageToServoAngle(solveBaseLinkageAngleDegrees(extensionLength))
    }
    override fun periodic() {


        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Extension: Telemetry Enabled")
            robot.telemetry.addData("analog voltage", "%3f".format(analogOutput.voltage))
            robot.telemetry.addData("measured servo angle", "%1f".format(getActualServoAngleFromAnalogSensor()))
            robot.telemetry.addData("extension length (inches)", targetLength)
            robot.telemetry.addData("servo angle (deg)", servo.angle)
            robot.telemetry.addData("servo raw position (0.0-1.0)", servo.position)
            robot.telemetry.addData("base linkage angle (deg)", solveBaseLinkageAngleDegrees(targetLength))
            robot.telemetry.update()
        }
    }

    private var closeEnoughToTargetLength = 10.0 // degrees measured servo angle
    fun isCloseEnoughToTargetLength() : Boolean {
        return false //abs(servo.angle - getActualServoAngleFromAnalogSensor()) < closeEnoughToTargetLength
    }
    private fun solveBaseLinkageAngleDegrees(linkageExtension  : Double) : Double {
        val mmToInchesConversion = 0.03937008
        // center to center distance of the two linkages
        val baseLinkageLength_mm = 175.0
        val secondaryLinkageLength_mm = 250.0
        val minimumLinkageToLinkageDistance = (secondaryLinkageLength_mm - baseLinkageLength_mm) * mmToInchesConversion
        val maximumLinkageToLinkageDistance = (secondaryLinkageLength_mm + baseLinkageLength_mm) * mmToInchesConversion
        val linkageToLinkageDistanceForSolve = clamp(minimumLinkageToLinkageDistance + linkageExtension,
            minimumLinkageToLinkageDistance, maximumLinkageToLinkageDistance)
        val l1 = baseLinkageLength_mm * mmToInchesConversion
        val l2 = secondaryLinkageLength_mm * mmToInchesConversion
        val initialGuess = Math.PI / 2.0
        var thetaRadians = initialGuess
        val solveTolerance = Math.PI / 180.0 // 1 degree is good enough

        // Newton's Method Solve
        for (i in 1..10) {
            val f =
                cos(thetaRadians) * l1 + sqrt(l2.pow(2) - (sin(thetaRadians) * l1).pow(2)) - linkageToLinkageDistanceForSolve
            val df =
                -sin(thetaRadians) * l1 - (l1.pow(2) * sin(thetaRadians) * cos(thetaRadians)) / sqrt(
                    l2.pow(2) - (sin(thetaRadians) * l1).pow(2)
                )
            val thetaNext = thetaRadians - f / df

            if (abs(thetaNext - thetaRadians) < solveTolerance) {
                thetaRadians = thetaNext
                break
            }
            thetaRadians = thetaNext
        }
        // we want the exterior angle
        return 180.0 - Math.toDegrees(thetaRadians)
    }
    private fun baseLinkageToServoAngle(baseLinkageAngle : Double) : Double {
        val gearRatio = 2.0
        val servoAngleAtZerobaseLinkageAngle = EXTENSION_SERVO_ANGLE_AT_ZERO_BASE_ANGLE
        return (baseLinkageAngle * gearRatio) - servoAngleAtZerobaseLinkageAngle
    }
    private fun getActualServoAngleFromAnalogSensor() : Double {
        val x1 = 0.671
        val y1 = 111.55
        val x2 = 3.109
        val y2 = -157.9
        val analogVoltage = analogOutput.voltage
        val slope = (y2 - y1) / (x2 - x1)
        val interpolatedServoAngle = slope * (analogVoltage - x1) + y1
        return interpolatedServoAngle
    }
}