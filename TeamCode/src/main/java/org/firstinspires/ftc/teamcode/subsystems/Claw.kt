package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.utilities.AxonServo
import org.firstinspires.ftc.teamcode.Cons.CLAW_CLOSE
import org.firstinspires.ftc.teamcode.Cons.CLAW_OPEN

class Claw(private val servo: AxonServo, val beamBreak: DigitalChannel): SubsystemBase() {
    var holdingCone = beamBreak.state
    override fun periodic(){
        holdingCone = beamBreak.state

    }
    fun openClaw(){
        servo.servo.position = (CLAW_OPEN - 500)/2000
    }
    fun closeClaw(){
        servo.servo.position = (CLAW_CLOSE - 500)/2000
    }
    init {
        register()
    }
}