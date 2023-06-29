package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.utilities.AxonServo
import org.firstinspires.ftc.teamcode.utilities.Constants.CLAW_CLOSE
import org.firstinspires.ftc.teamcode.utilities.Constants.CLAW_OPEN

class Claw(private val servo: AxonServo, val beamBreak: DigitalChannel): SubsystemBase() {
    var holdingCone = beamBreak.state
    override fun periodic(){
        holdingCone = beamBreak.state
    }
    fun openClaw(){
        servo.setPosition(CLAW_OPEN)
    }
    fun closeClaw(){
        servo.setPosition(CLAW_CLOSE)
    }
    init {
        register()
    }
}