package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt

class DiffWrist(
    private val leftServo: AxonCRServo,
    private val rightServo: AxonCRServo,
    val robot: Robot
) : SubsystemBase() {
    private val odometry: DifferentialOdometry =
        DifferentialOdometry({ leftServo.position }, { rightServo.position }, 0.035)
    var translation = Translation2d()
    var rotation = 0.0
    var goalTranslation = 0.0
    var goalRotation = 0.0

    override fun periodic() {
        /* Each servo will calculate its loop time by storing the degrees travelled
        *  since the last time update() was called */
        leftServo.update()
        rightServo.update()
        odometry.updatePose()

        val currentPose = odometry.pose
        translation = currentPose.translation
        rotation = currentPose.heading

    }
}