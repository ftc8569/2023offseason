package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utilities.AxonServo
import org.firstinspires.ftc.teamcode.utilities.Mode

/*
* All subsystems have a reference to the robot that they are a member of
* This is so for safety functions (eg - "Don't go down until this other thing is out of the way")
* we don't have to pick and choose which other subsystems that subsystem is aware of. It can
* look at the state of any of the subsystems also on the robot
*/

class Robot(val hardwareMap: HardwareMap, t: Telemetry) : com.arcrobotics.ftclib.command.Robot() {
    init {
        val hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
    }

    val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, t)

    val drivetrain = DrivetrainSubsystem(this)

    val turret: TurretSubsystem = TurretSubsystem(this, MotorEx(hardwareMap, "turret"))

    private val extensionServo = AxonServo(hardwareMap, "extension", 500.0, 2500.0)
    val extension = ExtensionLinkageSubsystem(this, hardwareMap.get(ServoImplEx::class.java, "extension"))

    val elbow = ElbowSubsystem(this, MotorEx(hardwareMap, "leftElbow"), MotorEx(hardwareMap, "rightElbow"))

    val poleAlignerServo = hardwareMap.get(ServoImplEx::class.java, "aligner")
    val aligner: PoleAlignerSubsystem = PoleAlignerSubsystem(this, poleAlignerServo)

    private val leftDifferentialServo = AxonServo(hardwareMap, "leftWrist", 500.0, 2500.0,)
    private val rightDifferentialServo = AxonServo(hardwareMap, "rightWrist", 500.0, 2500.0)
    val wrist: DifferentialWristSubsystem = DifferentialWristSubsystem(this, leftDifferentialServo, rightDifferentialServo)

    private val clawServo = AxonServo(hardwareMap, "claw", 500.0, 2500.0)
    private val beamBreakDigitalChannel = hardwareMap.get(DigitalChannel::class.java, "beamBreak")
    val claw = ClawSubsystem(this, clawServo, beamBreakDigitalChannel )

    var mode: Mode = Mode.INTAKE
    var fallenCone = false
    val leds =  LEDSubsystem(hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkin"), this)

}