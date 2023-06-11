package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import java.util.concurrent.TimeUnit

class DiffWrist(private val leftServo: AxonCRServo, private val rightServo: AxonCRServo, val robot:Robot) : SubsystemBase() {
    val kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(0.035)
    val odometry: DifferentialDriveOdometry = DifferentialDriveOdometry(Rotation2d())
    var translation = 0.0
    var rotation = 0.0
    var goalTranslation = 0.0
    var goalRotation = 0.0


    override fun periodic(){
        /* Each servo will calculate its loop time by storing the degrees travelled
        *  since the last time updateVelocity() was called */
        leftServo.updateVelocity()
        rightServo.updateVelocity()

    }







}