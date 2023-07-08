package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants

class Drivetrain(val hw: HardwareMap, val robot: Robot): SubsystemBase() {
    private val frontR = MotorEx(hw, "frontR")
    private val frontL = MotorEx(hw, "frontL")
    private val backR = MotorEx(hw, "backR")
    private val backL = MotorEx(hw, "backL")
    val drive = MecanumDrive(frontL, frontR, backL, backR)

    private val imu: IMU = hw.get(IMU::
    class.java, "imu")
    private var parameters = IMU.Parameters(
        RevHubOrientationOnRobot(
            DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR
        )
    )

    init {
        imu.initialize(parameters)
        frontR.inverted = true
        frontL.inverted = true
        backR.inverted = true
        backL.inverted = true
    }

    fun getYaw(): Double{
        return imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
    }

    override fun periodic(){
        robot.t.addData("Beambreak",robot.claw.holdingCone)
        robot.t.update()
    }

    init {
        register()
    }
}