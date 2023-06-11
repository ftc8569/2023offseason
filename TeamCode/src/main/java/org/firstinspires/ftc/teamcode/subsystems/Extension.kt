package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.utilities.AxonServo

class Extension(private val servo: AxonServo, private val robot: Robot) : SubsystemBase() {

    var position: Double
        set(pos) {
            servo.setPosition(pos)
        }
        get() = servo.position

    init {
        register()
    }
}