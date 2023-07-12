package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.Robot
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DigitalChannel

@TeleOp
class BeamBreakTest : CommandOpMode() {
    override fun initialize() {
        val t = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)

        val beamBreak = hardwareMap.get(DigitalChannel::class.java, "beamBreak")
        beamBreak.mode = DigitalChannel.Mode.INPUT
        t.addLine("Beam Break Test")
        t.update()

        schedule(MonitorBeamBreak(beamBreak, t))
    }
}

class MonitorBeamBreak(private val beamBreak : DigitalChannel, private val telemetry: MultipleTelemetry) : CommandBase() {
    override fun execute() {
        telemetry.addData("Beam Broken", !beamBreak.state)
        telemetry.update()
    }

    override fun isFinished(): Boolean {
        return false
    }
}