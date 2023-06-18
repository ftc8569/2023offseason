package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import org.firstinspires.ftc.teamcode.utilities.AxonServo

/*
* All subsystems have a reference to the robot that they are a member of
* This is so for safety functions (eg - "Don't go down until this other thing is out of the way")
* we don't have to pick and choose which other subsystems that subsystem is aware of. It can
* look at the state of any of the subsystems also on the robot
*/

class Robot(private val hw: HardwareMap) {
    val turret: Turret = Turret(MotorEx(hw, "turret"))
    val elbow: Elbow = Elbow(MotorEx(hw, "elbow1"), MotorEx(hw, "elbow2"), this)
//    val extension: Extension =
//        Extension(AxonCRServo(hw, "extension", "extensionAnalog", 500.0, 2500.0))
    val aligner: Aligner = Aligner(AxonServo(hw, "aligner", "alignerAnalog", 500.0, 2500.0), this)
//    val wrist: DiffWrist = DiffWrist()
}