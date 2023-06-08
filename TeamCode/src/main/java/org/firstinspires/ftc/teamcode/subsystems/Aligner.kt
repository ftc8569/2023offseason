package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.utilities.AxonServo

class Aligner(val servo: AxonServo, val extension: Extension) : SubsystemBase() {


    init {
        register()
    }
}