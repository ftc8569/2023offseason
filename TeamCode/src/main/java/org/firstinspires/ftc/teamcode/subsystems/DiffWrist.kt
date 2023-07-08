package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.apache.commons.math3.geometry.euclidean.threed.Rotation
import org.firstinspires.ftc.teamcode.Cons.*
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import org.firstinspires.ftc.teamcode.utilities.AxonServo
import kotlin.math.PI
import kotlin.math.abs

class DiffWrist(
    private val leftServo: AxonServo,
    private val rightServo: AxonServo,
    val telemetry: MultipleTelemetry,
) : SubsystemBase() {
    var twist:Double = 0.0
    var bend:Double = 0.0
    var leftAng: Double = 1500.0
    var rightAng:Double = 1500.0
    override fun periodic() {
        // Difference of 1000 micro_s = 90 deg
        leftAng = 0.5 + ((bend/90)*1000 - ((twist/90 * 1000)/2))/2000
        rightAng = 0.5 + ((bend/90)*1000 + ((twist/90 * 1000)/2))/2000

        leftServo.servo.position = leftAng
        rightServo.servo.position = 1.0 - rightAng
    }

    init {
        register()
    }
}