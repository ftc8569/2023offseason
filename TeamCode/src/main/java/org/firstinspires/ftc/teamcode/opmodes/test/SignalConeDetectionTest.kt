package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.general.UpdateTelemetry
import org.firstinspires.ftc.teamcode.commands.vision.DetectSignalCone
import org.firstinspires.ftc.teamcode.subsystems.Robot

@TeleOp
class SignalConeDetectionTest() : CommandOpMode() {
    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)
        schedule(
            DetectSignalCone(robot)
                .andThen(UpdateTelemetry(robot){ telemetry ->
                    telemetry.addData("Detected Cone", robot.detectedSignalCone)
                })
        )
    }
}