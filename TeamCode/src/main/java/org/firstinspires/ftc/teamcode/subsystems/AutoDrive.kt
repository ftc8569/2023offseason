package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive

class AutoDrive(val hw: HardwareMap): SubsystemBase() {
    val drive = SampleMecanumDrive(hw)

    init {
        register()
    }
}