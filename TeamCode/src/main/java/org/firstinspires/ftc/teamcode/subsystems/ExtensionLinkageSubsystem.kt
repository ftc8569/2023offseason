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


    private val velocity = 50.0 // inches / second
    private val acceleration = 30.0 // inches / second ^ 2

    val isExtended : Boolean
        get() = servo.servo.position > EXTENSION_HOME


    // set this to do motion profiling
    var extensionTargetInches: Double = 0.0

    private var currentExtensionDistanceInches: Double = 0.0

    private var previousTargetExtensionInches: Double = 0.0

    // set this directly to make the servo go to that position
    var actualPositionExtensionInches: Double = 0.0
        set(value) {
            position = angleRadiansToServoPosition(solveTheta(value))
        }

    private var mProfile = MotionProfileGenerator.generateMotionProfile(
        MotionState(extensionTargetInches, 0.0, 0.0),
        MotionState(extensionTargetInches, 0.0, 0.0),
        { velocity },
        { acceleration },
    )

    var timer = ElapsedTime()

    private var position : Double
        get() = servo.servo.position
        set(value) {
            servo.servo.position = value
        }

    fun home(){
        servo.servo.position = EXTENSION_HOME
    }

    override fun periodic() {
        generateMotionProfile(extensionTargetInches, actualPositionExtensionInches)
        currentExtensionDistanceInches = mProfile[timer.seconds()].x
        actualPositionExtensionInches = solveTheta(currentExtensionDistanceInches)
    }

    private fun solveTheta(linkage_length_inches: Double): Double {
        val initialGuess = Math.PI / 2
        var theta = initialGuess
        val epsilon = 1E-10
        val mmToInchesConversion = 0.03937008
        val l1 = 175.0 * mmToInchesConversion
        val l2 = 250.0 * mmToInchesConversion

        for (i in 1..10) {
            val f = cos(theta) * l1 + sqrt(l2.pow(2) - (sin(theta) * l1).pow(2)) - linkage_length_inches
            val df = -sin(theta) * l1 - (l1.pow(2) * sin(theta) * cos(theta)) / sqrt(l2.pow(2) - (sin(theta) * l1).pow(2))
            val thetaNext = theta - f / df

            if (abs(thetaNext - theta) < epsilon) {
                theta = thetaNext
                break
            }

            theta = thetaNext
        }
        // we want the exterior angle
        return Math.toRadians(180.0) - theta
    }

    private fun angleRadiansToServoPosition(radian_angle : Double) : Double {
        return servo.getServoPositionFromAngleDegrees(Math.toDegrees(radian_angle * 2))
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