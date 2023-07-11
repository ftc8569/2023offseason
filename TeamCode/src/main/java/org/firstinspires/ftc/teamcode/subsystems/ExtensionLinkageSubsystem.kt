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


    companion object {
        const val MINIMUM_EXTENSION = 5.19
        const val GROUND = 5.19
        const val LOW = 8.0
        const val MID = 10.0
        const val HIGH = 16.0
        const val PICKUP = MINIMUM_EXTENSION
    }

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
        val a = 0.00111573
        val b = -0.03578329
        val c = 0.49251416
        val d = -0.51091554
        var  theta =  linkage_length_inches.pow(3.0) * a
                    + linkage_length_inches.pow(2.0) * b
                    + linkage_length_inches * c + d
        if (linkage_length_inches <= 0) {
            theta = 0.0
        }
        return Math.toRadians(180.0) - theta - Math.toRadians(90.0)
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