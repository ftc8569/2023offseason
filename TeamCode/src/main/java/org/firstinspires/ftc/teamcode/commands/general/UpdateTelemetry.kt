package org.firstinspires.ftc.teamcode.commands.general

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.round

class UpdateTelemetry(private val robot: Robot, private val addTelemetry : (Telemetry) -> Unit) : CommandBase() {

    override fun initialize() {
        super.initialize()
        addTelemetry(robot.telemetry)
        robot.telemetry.update()
    }

    override fun isFinished(): Boolean {
        return true
    }
}