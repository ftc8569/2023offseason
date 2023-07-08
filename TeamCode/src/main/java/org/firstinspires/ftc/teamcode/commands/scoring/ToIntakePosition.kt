package org.firstinspires.ftc.teamcode.commands.scoring

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Robot

class ToIntakePosition(private val r: Robot): CommandBase() {
    override fun initialize() {
//        addRequirements(r.elbow, r.extension, r.claw, r.wrist)
//        r.elbow.targetAngle = -40.0
        r.extension.length = 0.4
        r.claw.openClaw()
        r.wrist.bend = -10.0
        r.wrist.twist = 0.0
    }

//    override fun isFinished(): Boolean {
//        return r.elbow.atTargetPosition and r.extension.atTargetPosition
//    }

}