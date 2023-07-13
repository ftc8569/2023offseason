package org.firstinspires.ftc.teamcode.subsystems

import androidx.core.math.MathUtils.clamp
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.command.SubsystemBase
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
        const val MAXIMUM_EXTENSION = 16.0
        const val GROUND = 5.19
        const val LOW = 8.0
        const val MID = 10.0
        const val HIGH = MAXIMUM_EXTENSION
        const val PICKUP = MINIMUM_EXTENSION
    }

    var isTelemetryEnabled = false
    val servoSpeedEstimate = 0.115 //seconds per 60 degrees @ 6V (Axon Max)
    val completionTimer = ElapsedTime()
    private var estimatedTimeToComplete = 0.0 //seconds
    val maximumMovementTime = abs(servo.maximumAngle - servo.minimumAngle) * servoSpeedEstimate / 60.0 //seconds


    private val velocity = 2.0 // inches / second
    private val acceleration = 2.0 // inches / second ^ 2
    val isExtended : Boolean
        get() = servo.angle > servo.getAngleDegreesFromServoPosition(EXTENSION_HOME)
    var targetLength : Double = EXTENSION_HOME
        set(value) {
            field = clamp(value, MINIMUM_EXTENSION, HIGH)
            val previousServoAngle = servoAngle
            servoAngle = baseLinkageToServoAngle(solveBaseLinkageAngleDegrees(targetLength))
            val angleChange = kotlin.math.abs(servoAngle - previousServoAngle)
            estimatedTimeToComplete = (angleChange * servoSpeedEstimate / 60.0).coerceAtMost(maximumMovementTime)
            completionTimer.reset()
        }
    var servoAngle : Double
        private set (value) {
            servo.angle = value
        }
        get() = servo.angle

    override fun periodic() {
        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Extension: Telemetry Enabled")
            robot.telemetry.addData("extension length (inches)", targetLength)
            robot.telemetry.addData("servo angle (deg)", servo.angle)
            robot.telemetry.addData("base linkage angle (deg)", solveBaseLinkageAngleDegrees(targetLength))
            robot.telemetry.update()
        }
    }
    fun movementShouldBeComplete() : Boolean {
        return completionTimer.seconds() > estimatedTimeToComplete
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
        val servoAngleAtZerobaseLinkageAngle = 164.0
        return (baseLinkageAngle * gearRatio) - servoAngleAtZerobaseLinkageAngle
    }
}