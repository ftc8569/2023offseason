package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.kotlin.extensions.gamepad.whenInactive
import com.qualcomm.hardware.ams.AMSColorSensor.Wait
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Cons
import org.firstinspires.ftc.teamcode.commands.commandgroups.*
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveMecanumSnap
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.general.MonitorRobotTelemetry
import org.firstinspires.ftc.teamcode.commands.scoring.SetAligner
import org.firstinspires.ftc.teamcode.commands.turret.ControlTurretAngle
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.ArmState
import org.firstinspires.ftc.teamcode.subsystems.ArmStatePositionData
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class LokiTeleop : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)
        val driver = GamepadEx(gamepad1)
        val gunner = GamepadEx(gamepad2)

//        robot.elbow.isTelemetryEnabled = true

        schedule(MoveToTravel(robot))

        // drive with left joystick snapped to 0 degree heading
        val driveCommand = DriveMecanumSnap(
            robot.drivetrain, 0.0,
            { driver.leftY.pow(2) * sign(driver.leftY) },
            { driver.leftX.pow(2) * sign(driver.leftX) },
        )
        robot.drivetrain.defaultCommand = driveCommand

        // Move the turret to the angle to the right joystick
        val turretController = ControlTurretAngle(robot) {
            Vector2d(-driver.rightY, driver.rightX)
        }
        turretController.schedule(false)

        // turn on the telemetry monitoring of the robot
        MonitorRobotTelemetry(robot).schedule(false)


        val triggerThreshold = 0.2
        val driverRightTrigger = Trigger { driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > triggerThreshold }

        val driverRightBumper = driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
        val driverLeftBumper = driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)

        val driverTriangleButton = driver.getGamepadButton(GamepadKeys.Button.Y)
        val driverCircleButton = driver.getGamepadButton(GamepadKeys.Button.B)
        val driverSquareButton = driver.getGamepadButton(GamepadKeys.Button.X)
        val driverCrossButton = driver.getGamepadButton(GamepadKeys.Button.A)
        val driverDPADDownButton = driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)

        driverRightTrigger.whenActive(MoveToScoreHighJunction(robot)).whenInactive(DepositCone(robot))
        driverRightBumper.whenActive(MoveToScoreMediumJunction(robot)).whenInactive(DepositCone(robot))
        driverTriangleButton.whenActive(MoveToScoreLowJunction(robot)).whenInactive(DepositCone(robot))
        driverCircleButton.whenActive(MoveToScoreGroundJunction(robot)).whenInactive(DepositCone(robot))
        driverDPADDownButton.whenActive(MoveToTiltPole(robot)).whenInactive(MoveToTravel(robot))
        driverCrossButton.whenActive(MoveToCycle(robot)).whenInactive(IntakeCone(robot, ArmStatePositionData.INTAKE))
        driverSquareButton.whenActive(PickupCone(robot))



        driverLeftBumper.whenActive(IntakeCone(robot, ArmStatePositionData.INTAKE))
        val beamBreakInIntakeMode = Trigger { robot.claw.holdingCone && robot.armState == ArmState.INTAKE }
        beamBreakInIntakeMode.whenActive(PickupCone(robot))

        val driverLeftTriggerConeRighting = Trigger { driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > triggerThreshold && !robot.claw.holdingCone }
        driverLeftTriggerConeRighting.whenActive(MoveToConeRighting(robot))
                .whenInactive(
                        SequentialCommandGroup(
                                SetElbowAngle(robot.elbow, ArmStatePositionData.TRAVEL.elbow.angle),
                                MoveToTravel(robot)
                        ))

        // drive DPAD left/right to adjust the heading
        val driverDPADLeftButton = driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val driverDPADRightButton = driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
        val driverDPADUpButton = driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)

        driverDPADRightButton.whenActive(InstantCommand( { driveCommand.adjustHeading( +3.0) }))
        driverDPADLeftButton.whenActive(InstantCommand( { driveCommand.adjustHeading( -3.0) }))
        driverDPADUpButton.whenActive(InstantCommand( { driveCommand.resetHeadingToImuHeading() }))
//        driverDPADDownButton.whenActive(InstantCommand( { driveCommand.isHeadingLocked = false }))
//            .whenInactive(InstantCommand( {
//                driveCommand.resetGoalHeadingToCurrentHeading()
//                driveCommand.isHeadingLocked = true
            }


    }
