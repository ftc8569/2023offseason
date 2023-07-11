package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import org.firstinspires.ftc.teamcode.utilities.Mode

class LEDSubsystem(private val robot: Robot, private val ledDriver: RevBlinkinLedDriver): SubsystemBase() {

    private val defaultPattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET
    init {
        register()
    }

    override fun periodic(){
        // TODO: here we can add an indication of the current mode of the robot
    }

    public fun updatePatternForHoming() {
        val pattern = when(robot.robotState) {
            RobotState.NOT_HOMED -> {
                RevBlinkinLedDriver.BlinkinPattern.RED
            }
            else -> {
                defaultPattern
            }
        }

        ledDriver.setPattern(pattern)
    }
}