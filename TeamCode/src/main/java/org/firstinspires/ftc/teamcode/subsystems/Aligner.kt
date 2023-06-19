package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.utilities.AxonServo

class Aligner(val servo: AxonServo, private val robot: Robot) : SubsystemBase() {
    init {
        register()
    }
}