package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.utilities.AxonServo

class Extension(private val servo: AxonServo) : SubsystemBase() {
    var position = 0.0
        set(pos) = servo.setPosition(pos)

    init {
        register()
    }
}