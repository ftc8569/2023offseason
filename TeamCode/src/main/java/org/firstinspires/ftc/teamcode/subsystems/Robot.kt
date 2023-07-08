package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import org.firstinspires.ftc.teamcode.utilities.AxonServo
import org.firstinspires.ftc.teamcode.utilities.Mode
import org.firstinspires.ftc.teamcode.utilities.PoleState

/*
* All subsystems have a reference to the robot that they are a member of
* This is so for safety functions (eg - "Don't go down until this other thing is out of the way")
* we don't have to pick and choose which other subsystems that subsystem is aware of. It can
* look at the state of any of the subsystems also on the robot
*/

class Robot(val hw: HardwareMap, val telemetry: Telemetry) {
    init {
        val hubs = hw.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
    }
    val t = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)

    val drivetrain = RRDrivetrain(hw, this)
    val turret: Turret = Turret(MotorEx(hw, "turret"))
    val extension: Extension =
        Extension(AxonCRServo(hw, "extension", "extension", 500.0, 2500.0), this)
    val elbow = Elbow(MotorEx(hw, "leftElbow"), MotorEx(hw, "rightElbow"), this)
//    val aligner: Aligner = Aligner(AxonServo(hw, "aligner", "alignerAnalog", 500.0, 2500.0), this)

    val wrist: DiffWrist = DiffWrist( AxonServo(hw, "leftWrist", 500.0, 2500.0),
        AxonServo(hw, "rightWrist", 500.0, 2500.0),
        t)
    val claw = Claw(AxonServo(hw, "claw", 500.0, 2500.0), hw.get(DigitalChannel::class.java, "beamBreak"))
//    var mode: Mode = Mode.INTAKE
//    val poleState = PoleState(0.0,0.0)



}