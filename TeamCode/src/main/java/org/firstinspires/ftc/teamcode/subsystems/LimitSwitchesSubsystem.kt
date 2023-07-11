package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DigitalChannel

class LimitSwitchesSubsystem(val robot : Robot, val leftLimitSwitch : DigitalChannel, val rightLimitSwitch : DigitalChannel) : SubsystemBase() {
    init {
        leftLimitSwitch.mode = DigitalChannel.Mode.INPUT
        rightLimitSwitch.mode = DigitalChannel.Mode.INPUT
        register()
    }
    var isTelemetryEnabled = false
    val leftLimitSwitchState : Boolean
        get() = leftLimitSwitch.state
    val rightLimitSwitchState : Boolean
        get() = rightLimitSwitch.state

    override fun periodic() {
        if(isTelemetryEnabled) {
            robot.telemetry.addData("Left Limit Switch:", leftLimitSwitch.state)
            robot.telemetry.addData("Right Limit Switch:", rightLimitSwitch.state)
            robot.telemetry.update()
        }
    }

}