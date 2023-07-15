package org.firstinspires.ftc.teamcode.commands.commandgroups

import com.arcrobotics.ftclib.command.*
import org.firstinspires.ftc.teamcode.Cons
import org.firstinspires.ftc.teamcode.commands.claw.ClawRegripCone
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.general.ConfigurableCommandBase
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ArmStatePositionData
import org.firstinspires.ftc.teamcode.subsystems.Robot

class IntermediateBackToCones(val robot : Robot) : ConfigurableCommandBase()  {
    override fun initialize() {
        super.initialize()
        robot.armState = ArmState.TRAVEL
    }

    override fun configure(): CommandBase {
        return SequentialCommandGroup(
            ParallelCommandGroup(
                SetWristAngles(robot.wrist,
                    ArmStatePositionData.INTERMEDIATE.wrist.bendAngle,
                    ArmStatePositionData.INTERMEDIATE.wrist.twistAngle),
                SetExtensionLinkage(robot.extension, ArmStatePositionData.INTERMEDIATE.extension.length, Cons.EASING),
                SetElbowAngle(robot.elbow, ArmStatePositionData.INTERMEDIATE.elbow.angle),
                SetTurretAngle(robot.turret, 54.0)
            ),
            SetAligner(robot.aligner, ArmStatePositionData.INTERMEDIATE.aligner.angle),
            ConditionalCommand(ClawRegripCone(robot), InstantCommand({ println("MoveToIntermediateScorePos - Skipped Regrip")})) { robot.claw.holdingCone },
        )
    }
}