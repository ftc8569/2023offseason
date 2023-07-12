package org.firstinspires.ftc.teamcode.commands.general

import com.arcrobotics.ftclib.command.CommandBase

/**
 * This class is used to create a command that can be configured just prior command.initialize() time from a lambda.
 */
class ConfigurableCommand(private val commandGenerator : () -> CommandBase) : ConfigurableCommandBase() {
    override fun configure(): CommandBase {
        return commandGenerator.invoke()
    }
}