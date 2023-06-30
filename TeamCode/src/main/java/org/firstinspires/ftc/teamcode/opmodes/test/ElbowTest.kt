package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.subsystems.Elbow

class ElbowTest: LinearOpMode() {
    override fun runOpMode() {
        val elbow = Elbow(MotorEx(hardwareMap, "leftElbow"), MotorEx(hardwareMap, "rightElbow"))
    }
}