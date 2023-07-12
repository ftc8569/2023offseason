package org.firstinspires.ftc.teamcode.subsystems

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


    var isTelemetryEnabled = false
    private val velocity = 2.0 // inches / second
    private val acceleration = 2.0 // inches / second ^ 2

    companion object {
        const val MINIMUM_EXTENSION = 0.0
        const val GROUND = 5.19
        const val LOW = 8.0
        const val MID = 10.0
        const val HIGH = 16.0
        const val PICKUP = MINIMUM_EXTENSION
    }

    val isExtended : Boolean
        get() = servo.angle > servo.getAngleDegreesFromServoPosition(EXTENSION_HOME)
    var extensionTargetInches: Double = 0.0
    private var currentExtensionDistanceInches: Double = 0.0
    private var previousTargetExtensionInches: Double = 0.0
    var extensionLength : Double = EXTENSION_HOME
    private var mProfile = MotionProfileGenerator.generateMotionProfile(
        MotionState(extensionTargetInches, 0.0, 0.0),
        MotionState(extensionTargetInches, 0.0, 0.0),
        { velocity },
        { acceleration },
    )
    var timer = ElapsedTime()

    override fun periodic() {
//        generateMotionProfile(extensionTargetInches, extensionLength)
//        currentExtensionDistanceInches = mProfile[timer.seconds()].x
//        extensionLength = currentExtensionDistanceInches

        servo.angle = baseLinkageToServoAngle(solveBaseLinkageAngleDegrees(extensionLength))

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Extension: Telemetry Enabled")
            robot.telemetry.addData("extension length (inches)", extensionLength)
            robot.telemetry.addData("servo angle (deg)", servo.angle)
            robot.telemetry.addData("base linkage angle (deg)", solveBaseLinkageAngleDegrees(extensionLength))
            robot.telemetry.addData("motion profile distance", currentExtensionDistanceInches)
            robot.telemetry.update()
        }
    }

    private val minimumExtensionSolveLength = 5.0
    private val thetaAtMinimumExtensionSolveLength = solveBaseLinkageAngleDegrees(minimumExtensionSolveLength)
    private val maximumExtensionSolveLength = 15.0
    private val thetaAtMaximumExtensionSolveLength = solveBaseLinkageAngleDegrees(maximumExtensionSolveLength)
    private fun solveBaseLinkageAngleDegrees(linkage_length_inches: Double): Double {

        return if (linkage_length_inches < minimumExtensionSolveLength) {
            interpolateBaseLinkageAngle(linkage_length_inches)
        }
        else if (linkage_length_inches > maximumExtensionSolveLength) {
            thetaAtMaximumExtensionSolveLength
        }
        else {
            val mmToInchesConversion = 0.03937008
            // center to center distance of the two linkages
            val baseLinkageLength_mm = 175.0
            val secondaryLinkageLength_mm = 250.0
            val l1 = baseLinkageLength_mm * mmToInchesConversion
            val l2 = secondaryLinkageLength_mm * mmToInchesConversion
            val initialGuess = Math.PI / 2
            var thetaRadians = initialGuess
            val solveTolerance = Math.PI/180.0 // 1 degree is good enough

            // Newton's Method Solve
            for (i in 1..10) {
                val f = cos(thetaRadians) * l1 + sqrt(l2.pow(2) - (sin(thetaRadians) * l1).pow(2)) - linkage_length_inches
                val df = -sin(thetaRadians) * l1 - (l1.pow(2) * sin(thetaRadians) * cos(thetaRadians)) / sqrt(l2.pow(2) - (sin(thetaRadians) * l1).pow(2))
                val thetaNext = thetaRadians - f / df

                if (abs(thetaNext - thetaRadians) < solveTolerance) {
                    thetaRadians = thetaNext
                    break
                }
                thetaRadians = thetaNext
            }

            // we want the exterior angle
            180.0 - Math.toDegrees(thetaRadians)
        }
    }
    fun interpolateBaseLinkageAngle(length: Double): Double {
        if(length < 5.0 && length > 0.0) {
            val slope = thetaAtMinimumExtensionSolveLength / minimumExtensionSolveLength
            return slope * length
        }
        else
            throw IllegalArgumentException("Length must be between 0 and 5 inches")
    }

    private fun baseLinkageToServoAngle(baseLinkageAngle : Double) : Double {
        val gearRatio = 2.0
        val servoAngleAtZerobaseLinkageAngle = 164.0
        // Note the servo angle is reversed from the base linkage angle
        return servoAngleAtZerobaseLinkageAngle - (baseLinkageAngle * gearRatio)
    }

    private fun generateMotionProfile(reference: Double, state: Double) {
        if (previousTargetExtensionInches == reference) {
            return
        }
        previousTargetExtensionInches = reference
        mProfile = MotionProfileGenerator.generateMotionProfile(
            MotionState(state, 0.0, 0.0),
            MotionState(reference, 0.0, 0.0),
            { velocity },
            { acceleration },
        )
        timer.reset()
    }

}