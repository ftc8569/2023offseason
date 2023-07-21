package org.firstinspires.ftc.teamcode.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.commandgroups.MoveToTravel
import org.firstinspires.ftc.teamcode.commands.general.UpdateTelemetry
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.commands.vision.DetectSignalCone
import org.firstinspires.ftc.teamcode.commands.vision.DetectSignalConfigurable
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.AlliancePosition
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.DepositHighPoleAuto
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.IntakeFromConeStack
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.MoveToAutoScoringPosition
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.Park
import org.firstinspires.ftc.teamcode.subsystems.*

@Autonomous
class LokiAutoRightSide() : CommandOpMode() {

    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry, OpModeType.AUTONOMOUS)
        val alliancePosition = AlliancePosition.RIGHT

        // Coordinate System: +x is forward (away from driver station), +y is left, +theta is counter-clockwise
        // (0,0) is the center of the field.  0.0 radians heading is directly away from the driver station along the +x axis
        // this is where we map the robot coordinate system to the field coordinate system
        val startPose = Pose2d(-87.5, -60.0, 0.0)
        robot.drivetrain.poseEstimate = startPose

        // Move the wrist/claw/extension to the right starting position
        // open the claw waiting for a preloaded cone.  The claw will close when the cone is detected and open if is not detected
        robot.extension.targetLength = ArmStatePositionData.ARM_HOME.extension.length
        robot.wrist.bendAngleDegrees = ArmStatePositionData.ARM_HOME.wrist.bendAngle
        robot.wrist.twistAngleDegrees = ArmStatePositionData.ARM_HOME.wrist.twistAngle
        robot.claw.position = ClawPositions.OPEN_FOR_INTAKE

        // schedule the commands for Auto
        val autoCommands = SequentialCommandGroup(
            MoveToTravel(robot),
            DetectSignalCone(robot),
//            DetectSignalConfigurable(robot),
            UpdateTelemetry(robot){ telemetry -> telemetry.addData("Detected Cone", robot.detectedSignalCone)},
            MoveToAutoScoringPosition(robot, alliancePosition),
            DepositHighPoleAuto(robot, alliancePosition),
            IntakeFromConeStack(robot, alliancePosition, 5),
            DepositHighPoleAuto(robot, alliancePosition),
            IntakeFromConeStack(robot, alliancePosition, 4),
            DepositHighPoleAuto(robot, alliancePosition),
//            IntakeFromConeStack(robot, alliancePosition, 3),
//            DepositHighPoleAuto(robot, alliancePosition),
//            IntakeFromConeStack(robot, alliancePosition, 2),
//            DepositHighPoleAuto(robot, alliancePosition),
//            IntakeFromConeStack(robot, alliancePosition, 1),
//            DepositHighPoleAuto(robot, alliancePosition),
            MoveToTravel(robot),
            ParallelCommandGroup(
                SetTurretAngle(robot.turret, 0.0),
                Park(robot, alliancePosition)
            )
        )

        schedule(autoCommands)

        while (robot.claw.position != ClawPositions.HOLD_CONE) {
            if(robot.claw.holdingCone)
                robot.claw.position = ClawPositions.HOLD_CONE
            sleep(100)
        }
    }
}