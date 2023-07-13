package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utilities.AxonServo
import kotlin.math.abs

enum class ClawPositions {
    OPEN_FOR_INTAKE,
    HOLD_CONE,
    RELEASE_CONE_BUT_HOLD_TSE,
    MANUAL
}

class ClawSubsystem(val robot : Robot, val servo: AxonServo, val beamBreak: DigitalChannel): SubsystemBase() {
    init {
        register()
        beamBreak.mode = DigitalChannel.Mode.INPUT
    }
    val servoSpeedEstimate = 0.09 //seconds per 60 degrees @ 6V (Axon Minis)
    val timer = ElapsedTime()
    private var estimatedTimeToComplete = 0.0 //seconds
    val maximumMovementTime = abs(servo.maximumAngle - servo.minimumAngle) * servoSpeedEstimate / 60.0 //seconds

    val holdingCone: Boolean
        get() = !beamBreak.state
    var position:ClawPositions = ClawPositions.HOLD_CONE
        set(value) {
            val previousAngle = angle
            angle  = getAngleForPosition(value)
            val angleChange = kotlin.math.abs(angle - previousAngle)
            estimatedTimeToComplete = (angleChange * servoSpeedEstimate / 60.0).coerceAtMost(maximumMovementTime)
            timer.reset()
            field = value
        }
    var angle : Double
        private set (value) {
            servo.angle = value
        }
        get() = servo.angle

    var isTelemetryEnabled = false

    private fun getAngleForPosition(position : ClawPositions) : Double = when (position) {
        ClawPositions.OPEN_FOR_INTAKE ->  ArmStatePositionData.CLAW_OPEN_FOR_INTAKE.angle
        ClawPositions.HOLD_CONE ->  ArmStatePositionData.CLAW_HOLD_CONE.angle
        ClawPositions.RELEASE_CONE_BUT_HOLD_TSE -> ArmStatePositionData.CLAW_RELEASE_CONE_BUT_HOLD_TSE.angle
        ClawPositions.MANUAL -> servo.angle
    }

    override fun periodic(){
        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Claw: Telemetry Enabled")
            robot.telemetry.addData("claw position", position)
            robot.telemetry.addData("claw angle", servo.angle)
            robot.telemetry.addData("beam break", !beamBreak.state)
            robot.telemetry.update()
        }
    }

    fun movementShouldBeComplete() : Boolean {
        return timer.seconds() > estimatedTimeToComplete
    }
}