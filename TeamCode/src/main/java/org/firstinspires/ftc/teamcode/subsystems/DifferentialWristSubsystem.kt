package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.utilities.AxonServo

class DifferentialWristSubsystem( val robot: Robot, private val leftServo: AxonServo, private val rightServo: AxonServo) : SubsystemBase() {

    var twistAngleDegrees : Double = 0.0
    var bendAngleDegrees : Double = 0.0
    var isTelemetryEnabled = false

    init {
        // reverse the right servo
        register()
    }

    override fun periodic() {
        var leftServoAngle = bendAngleDegrees - twistAngleDegrees / 2.0
        var rightServoAngle = bendAngleDegrees + twistAngleDegrees /2.0

        leftServo.servo.position = leftServo.getServoPositionFromAngleDegrees(leftServoAngle)
        rightServo.servo.position = 1.0 - rightServo.getServoPositionFromAngleDegrees(rightServoAngle)

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Differential Wrist: Telemetry Enabled")
            robot.telemetry.addData("Bend Angle:", bendAngleDegrees)
            robot.telemetry.addData("Twist Angle:", twistAngleDegrees)

            robot.telemetry.addData("Left Servo Angle:", leftServoAngle)
            robot.telemetry.addData("Right Servo Angle:", rightServoAngle)

            robot.telemetry.addData("Left Servo Position:", leftServo.servo.position)
            robot.telemetry.addData("Right Servo Position:", rightServo.servo.position)

            robot.telemetry.addData("Left Servo Pulse Width:", leftServo.getServoPulseWidthFromPosition(leftServo.servo.position))
            robot.telemetry.addData("Right Servo Pulse Width:", rightServo.getServoPulseWidthFromPosition(rightServo.servo.position))

            robot.telemetry.update()
        }
    }

}