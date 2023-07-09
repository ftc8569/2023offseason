package org.firstinspires.ftc.teamcode.commands.turret

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.HelperFunctions

class MaintainAngle(val r: Robot): CommandBase() {
    init {
        addRequirements(r.turret)
    }

    override fun execute(){
        r.turret.targetAngle = HelperFunctions.toDegrees(
                HelperFunctions.toRobotRelativeAngle(
                    HelperFunctions.toRadians(r.turret.fieldRelativeTargetAngle),
                    HelperFunctions.toRadians(r.drivetrain.getYaw())
                )
            )
    }

    override fun isFinished() = false
}