package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.wrist.WristTrajectory
import org.firstinspires.ftc.teamcode.subsystems.DiffWrist
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import kotlin.math.PI

@TeleOp
class WristTest : LinearOpMode() {
    override fun runOpMode() {
        val scheduler = CommandScheduler.getInstance()
        val wrist = DiffWrist(
            AxonCRServo(hardwareMap, "leftWrist", "leftWrist", 500.0, 2500.0),
            AxonCRServo(hardwareMap, "rightWrist", "rightWrist", 500.0, 2500.0),
            MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        )

        val gp1 = GamepadEx(gamepad1)
        val start = Pose2d(0.0,0.0, Rotation2d(0.0))
        val end = Pose2d(0.001, 0.001, Rotation2d(PI))

        waitForStart()
        while (opModeIsActive() && !isStopRequested) {
            scheduler.run()
            gp1.readButtons()

            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                scheduler.schedule(WristTrajectory(wrist, start, end))
            } else if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                scheduler.schedule(WristTrajectory(wrist, end, start))
            }

        }
        scheduler.reset()
    }
}