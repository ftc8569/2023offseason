package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import org.firstinspires.ftc.teamcode.utilities.Mode

class LEDs(private val driver: RevBlinkinLedDriver, val r: Robot): SubsystemBase() {

    var pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET

    override fun periodic(){
//        pattern = if(r.mode == Mode.INTAKE){
//            RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET
//        } else {
//            RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;
//        }
//        driver.setPattern(pattern)
    }
    init {
        register()
    }

}