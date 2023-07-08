package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.TurretLock45
import org.firstinspires.ftc.teamcode.commands.TurretLock90
import org.firstinspires.ftc.teamcode.commands.scoring.ToIntakePosition
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.HelperFunctions
import org.firstinspires.ftc.teamcode.utilities.Mode
import kotlin.math.PI
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class TeleopV1 : CommandOpMode() {
    override fun initialize() {
        val driver = GamepadEx(gamepad1)
        val gunner = GamepadEx(gamepad2)
        val r = Robot(hardwareMap, telemetry)

//        r.drivetrain.defaultCommand = InstantCommand({
//            r.drivetrain.driveFieldCentric(
//                driver.leftY.pow(2) * sign(driver.leftY),
//                driver.leftX.pow(2) * sign(driver.leftX),
//                driver.rightX.pow(4) * sign(driver.rightX),
//                r.drivetrain.localizer.poseEstimate.heading * 180 / PI
//            )
//        }, r.drivetrain)

        // Turret should maintain field relative angle
//        r.turret.defaultCommand = InstantCommand({
//            r.turret.targetAngle = HelperFunctions.toDegrees(
//                HelperFunctions.toRobotRelativeAngle(
//                    HelperFunctions.toRadians(r.turret.targetAngle),
//                    r.drivetrain.localizer.poseEstimate.heading
//                )
//            )
//        }, r.turret)

//        Trigger { gunner.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 }.whenActive(
//            ConditionalCommand(
//                TurretLock45(
//                    r.turret,
//                    { r.drivetrain.poseEstimate.heading },
//                    { Vector2d(gunner.leftX, gunner.leftY) }),
//                TurretLock90( r.turret,
//                    { r.drivetrain.poseEstimate.heading },
//                    { Vector2d(gunner.leftX, gunner.leftY) })
//            ) { r.mode == Mode.SCORE }
//        )
//
//        gunner.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//            ConditionalCommand(
//                ToIntakePosition(r),
//                ParallelCommandGroup(
//                    InstantCommand({r.extension.length = 0.4}, r.extension),
//                    InstantCommand({r.wrist.bend = 20.0; r.wrist.twist = if (r.fallenCone) 90.0 else 0.0}, r.wrist),
//                    InstantCommand({r.elbow.targetAngle = 50.0}, r.elbow)
//                )
//            ) { r.mode == Mode.INTAKE }
//        )
//
//        gunner.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//            ConditionalCommand(
//                InstantCommand(
//                    {r.claw.openClaw(); r.mode = Mode.INTAKE}, r.claw
//                ),
//                InstantCommand(
//                    {r.claw.closeClaw(); r.mode = Mode.SCORE}, r.claw
//                )
//            ) { r.mode == Mode.SCORE }
//        )
//
//        gunner.getGamepadButton(GamepadKeys.Button.Y).whenPressed(InstantCommand({r.fallenCone = !r.fallenCone}))
//

    }
}