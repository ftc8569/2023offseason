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

class MinimalStowFromStackToPole(val robot: Robot) : ConfigurableCommandBase() {
    override fun initialize() {
        super.initialize()
        robot.armState = ArmState.TRAVEL
    }

    override fun configure(): CommandBase {
        return SequentialCommandGroup(
            ParallelCommandGroup(
                SetWristAngles(
                    robot.wrist,
                    ArmStatePositionData.CONESTACK_TO_HIGH_POLE.arm.wrist.bendAngle,
                    ArmStatePositionData.CONESTACK_TO_HIGH_POLE.arm.wrist.twistAngle
                ),
                SetExtensionLinkage(
                    robot.extension,
                    ArmStatePositionData.CONESTACK_TO_HIGH_POLE.arm.extension.length,
                    Cons.EASING
                ),
                SetElbowAngle(
                    robot.elbow,
                    ArmStatePositionData.CONESTACK_TO_HIGH_POLE.arm.elbow.angle
                ),
                SetTurretAngle(
                    robot.turret,
                    ArmStatePositionData.CONESTACK_TO_HIGH_POLE.turret.angle
                )
            ),
            SetAligner(
                robot.aligner,
                ArmStatePositionData.HIGH_POLE_TO_CONESTACK.arm.aligner.angle
            ),
            ConditionalCommand(
                ClawRegripCone(robot),
                InstantCommand({ println("MoveToIntermediateScorePos - Skipped Regrip") })
            ) { robot.claw.holdingCone },
        )
    }
}