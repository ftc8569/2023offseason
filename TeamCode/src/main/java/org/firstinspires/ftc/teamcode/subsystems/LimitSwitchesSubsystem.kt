package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DigitalChannel

class LimitSwitchesSubsystem(private val robot : Robot, private val leftLimitSwitch : DigitalChannel, private val rightLimitSwitch : DigitalChannel) : SubsystemBase() {
    init {
        leftLimitSwitch.mode = DigitalChannel.Mode.INPUT
        rightLimitSwitch.mode = DigitalChannel.Mode.INPUT
        register()
    }
    var isTelemetryEnabled = false
    val leftLimitSwitchActive : Boolean
        get() = !leftLimitSwitch.state
    val rightLimitSwitchActive  : Boolean
        get() = !rightLimitSwitch.state

    override fun periodic() {
        if(isTelemetryEnabled) {
            robot.telemetry.addData("Left Limit Switch:", leftLimitSwitchActive)
            robot.telemetry.addData("Right Limit Switch:", rightLimitSwitchActive)
            robot.telemetry.update()
        }
    }

    public fun isAtHome() : Boolean {
        return leftLimitSwitchActive || rightLimitSwitchActive
    }
    public fun isAtLeftHome() : Boolean {
        return leftLimitSwitchActive
    }
    public fun isAtRightHome() : Boolean {
        return rightLimitSwitchActive
    }
    public fun getHomePosition() : HomePosition {
        return if(isAtLeftHome()) {
            HomePosition.LEFT
        } else if (isAtRightHome()) {
            HomePosition.RIGHT
        } else {
            return HomePosition.NONE
        }
    }
    public fun getHomeAngles() : LimitSwitchHomeAngles? {
        return if(isAtLeftHome()) {
            LimitSwitchHomeAngles.LEFT_HOME
        } else if (isAtRightHome()) {
            LimitSwitchHomeAngles.RIGHT_HOME
        } else {
            return null
        }
    }
}
enum class HomePosition {
    LEFT, RIGHT, NONE
}
data class LimitSwitchHomeAngles(val turretAngle : Double, val elbowAngle : Double) {
    companion object {
        val LEFT_HOME = LimitSwitchHomeAngles(ArmStatePositionData.START_LEFT.turret.angle, ArmStatePositionData.START_LEFT.arm.elbow.angle)
        val RIGHT_HOME = LimitSwitchHomeAngles(ArmStatePositionData.START_RIGHT.turret.angle, ArmStatePositionData.START_RIGHT.arm.elbow.angle)
    }
}