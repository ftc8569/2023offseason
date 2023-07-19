package org.firstinspires.ftc.teamcode.commands.vision

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot

class DetectSignalConfigurable(val robot:Robot): ConfigurableCommandBase() {
    override fun configure(): CommandBase {
        return DetectSignalCone(robot)
    }
}