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
import org.firstinspires.ftc.teamcode.Cons.WRIST_LEFT_KP
import org.firstinspires.ftc.teamcode.Cons.WRIST_RIGHT_KP
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import kotlin.math.PI

class DiffWrist(
    private val leftServo: AxonCRServo,
    private val rightServo: AxonCRServo,
    val telemetry: MultipleTelemetry
//    val robot: Robot
) : SubsystemBase() {
    init {
        leftServo.servo.direction = DcMotorSimple.Direction.REVERSE
        leftServo.analogReversed = true
    }
    private val wheelRadius = 0.022
    private val leftInitial = leftServo.position
    private val rightInitial = rightServo.position
    val odo = DifferentialOdometry({ (leftServo.position - leftInitial)/360 * (wheelRadius * PI * 2) * (39.37)  },
        { (rightServo.position - rightInitial) /360 * (wheelRadius * PI * 2) * (39.37) },
        1.5
    )
    val kinematics = DifferentialDriveKinematics(0.038)
    var speeds = ChassisSpeeds(0.0,0.0,0.0)
    val leftController = PIDController(WRIST_LEFT_KP,0.0,0.0)
    val rightController = PIDController(WRIST_RIGHT_KP,0.0,0.0)

    override fun periodic() {
        /* Each servo will calculate its loop time by storing the degrees travelled
        *  since the last time update() was called */
        leftServo.update()
        rightServo.update()

        val wheelSpeeds = kinematics.toWheelSpeeds(speeds)
        val leftSpeed = wheelSpeeds.leftMetersPerSecond / wheelRadius
        val rightSpeed = wheelSpeeds.rightMetersPerSecond / wheelRadius

        val leftOut = leftController.calculate(leftServo.velocity.get(), leftSpeed)
        val rightOut = rightController.calculate(rightServo.velocity.get(), rightSpeed)

//        leftServo.setPower(leftOut)
//        rightServo.setPower(rightOut)

        telemetry.addData("Left velocity", leftServo.velocity.get())
        telemetry.addData("Right velocity", rightServo.velocity.get())
        telemetry.addData("Left Goal", leftSpeed)
        telemetry.addData("Right goal", rightSpeed)
        telemetry.addData("Angle", odo.pose.rotation.degrees)
        telemetry.addData("translation", odo.pose.translation)
        telemetry.addData("odo", odo)
        telemetry.update()

        odo.updatePose()
    }

    init {
        odo.updatePose(Pose2d(0.0,0.0, Rotation2d(0.0)))
        register()
    }
}