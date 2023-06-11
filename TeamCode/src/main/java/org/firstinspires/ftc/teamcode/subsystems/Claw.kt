package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.utilities.AxonServo

class Claw(val servo: AxonServo, val beamBreak: DigitalChannel): SubsystemBase() {
    var holdingCone = false
}