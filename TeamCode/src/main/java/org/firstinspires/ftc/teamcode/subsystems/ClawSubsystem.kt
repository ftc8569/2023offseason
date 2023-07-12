package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.Cons.*
import org.firstinspires.ftc.teamcode.utilities.AxonServo

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
    val holdingCone: Boolean
        get() = !beamBreak.state
    var position:ClawPositions = ClawPositions.HOLD_CONE
    var isTelemetryEnabled = false

    override fun periodic(){
        when (position) {
            ClawPositions.OPEN_FOR_INTAKE -> servo.angle = ArmStates.CLAW_OPEN_FOR_INTAKE.angle
            ClawPositions.HOLD_CONE -> servo.angle = ArmStates.CLAW_HOLD_CONE.angle
            ClawPositions.RELEASE_CONE_BUT_HOLD_TSE -> servo.angle = ArmStates.CLAW_RELEASE_CONE_BUT_HOLD_TSE.angle
            ClawPositions.MANUAL -> {}
        }
        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Claw: Telemetry Enabled")
            robot.telemetry.addData("claw position", position)
            robot.telemetry.addData("claw angle", servo.angle)
            robot.telemetry.addData("beam break", !beamBreak.state)
            robot.telemetry.update()
        }
    }
}