package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.round

class UpdateTelemetry(val robot: Robot, val message : String) : CommandBase() {

    override fun initialize() {
        super.initialize()
        robot.telemetry.addLine(message)
        robot.telemetry.update()
    }
}