package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.apriltags.ConeNumber
import org.firstinspires.ftc.teamcode.utilities.AxonServo
import java.lang.Thread.sleep

enum class OpModeType {
    TELEOP, AUTONOMOUS
}
enum class ArmState {
    SCORE_GROUND, SCORE_LOW, SCORE_MEDIUM, SCORE_HIGH, INTAKE, TRAVEL, START
}
enum class RobotState {
    NOT_HOMED, HOMED
}
class Robot(val hardwareMap: HardwareMap, t: Telemetry, val opModeType: OpModeType = OpModeType.TELEOP, private val isTelemetryEnabled : Boolean = false) : com.arcrobotics.ftclib.command.Robot() {

    init {
        val hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
    }
    var detectedSignalCone = ConeNumber.NONE
    var armState = ArmState.START
    set (value) {
        field = value
        if (isTelemetryEnabled) {
            telemetry.addData("Arm State", value)
            telemetry.update()
        }
    }

    var robotState = RobotState.NOT_HOMED
        private set

    companion object {
        var savedHomeResultsFromAuto : HomingResult? = null
    }

    val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, t)

    val leftLimitSwitch = hardwareMap.get(DigitalChannel::class.java, "LeftLimitSwitch")
    val rightLimitSwitch = hardwareMap.get(DigitalChannel::class.java, "RightLimitSwitch")
    val limitSwitches = LimitSwitchesSubsystem(this, leftLimitSwitch, rightLimitSwitch)

    val leds =  LEDSubsystem(this, hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkin"))

    private val homingResult : HomingResult = homeMecahnisms(limitSwitches, leds, telemetry)

    val drivetrain = DrivetrainSubsystem(this)
    val turret: TurretSubsystem = TurretSubsystem(this, MotorEx(hardwareMap, "turret"), homingResult)
    val elbow = ElbowSubsystem(this, MotorEx(hardwareMap, "leftElbow"), MotorEx(hardwareMap, "rightElbow"), homingResult)
    val signalSleeve = SignalConeDetectorSubSystem(this)

    private val extensionServo = AxonServo(hardwareMap, "extension", 500.0, 2500.0, 355.0, true)
    val extension = ExtensionLinkageSubsystem(this, extensionServo)

    private val poleAlignerServo = hardwareMap.get(ServoImplEx::class.java, "aligner")
    val aligner: PoleAlignerSubsystem = PoleAlignerSubsystem(this, poleAlignerServo)

    private val leftDifferentialServo = AxonServo(hardwareMap, "leftWrist", 500.0, 2500.0, 355.0)
    private val rightDifferentialServo = AxonServo(hardwareMap, "rightWrist", 500.0, 2500.0, 355.0, true )
    val wrist: DifferentialWristSubsystem = DifferentialWristSubsystem(this, leftDifferentialServo, rightDifferentialServo)

    private val clawServo = AxonServo(hardwareMap, "claw", 500.0, 2500.0)
    private val beamBreakDigitalChannel = hardwareMap.get(DigitalChannel::class.java, "beamBreak")
    val claw = ClawSubsystem(this, clawServo, beamBreakDigitalChannel )


    private fun homeMecahnisms(limitSwitches : LimitSwitchesSubsystem, leds : LEDSubsystem, telemetry: Telemetry) : HomingResult {
        leds.updatePatternForHoming()

        val homingResult  = when(opModeType) {
            OpModeType.AUTONOMOUS -> {
                // if we are initializing autonomous, we need homeMecahnisms on initialization every time
                waitForHome(telemetry)
                // and save these home angles for use in teleop
                savedHomeResultsFromAuto = HomingResult(limitSwitches.getHomeAngles()!!, limitSwitches.getHomePosition(), HomingMethod.LIMIT_SWITCH)
                savedHomeResultsFromAuto!!
            }
            OpModeType.TELEOP -> {
                val resultToUse = if(limitSwitches.isAtHome()) {
                    // if we are already at home, we don't need to homeMecahnisms
                    HomingResult(limitSwitches.getHomeAngles()!!, limitSwitches.getHomePosition(), HomingMethod.LIMIT_SWITCH)
                }
                else if(null != savedHomeResultsFromAuto) {
                    // if we are not at home, but we have a saved result from auto, use that
                    val result = HomingResult(  savedHomeResultsFromAuto!!.homeAngles,
                                                savedHomeResultsFromAuto!!.homePosition,
                                                HomingMethod.USING_SAVED_FROM_AUTO)
                    savedHomeResultsFromAuto = null // clear the saved result so that we only use the saved result once
                    result
                }
                else {
                    // if we are not at home, and we don't have a saved result from auto, homeMecahnisms
                    waitForHome(telemetry)
                    HomingResult(   limitSwitches.getHomeAngles()!!,
                                    limitSwitches.getHomePosition(),
                                    HomingMethod.LIMIT_SWITCH)
                }
                resultToUse
            }
        }
        if(isTelemetryEnabled) {
            telemetry.addData("method", homingResult.method)
            telemetry.addData("homePosition", homingResult.homePosition)
            telemetry.addData("turretAngle", homingResult.homeAngles.turretAngle)
            telemetry.addData("elbowAngle", homingResult.homeAngles.elbowAngle)
            telemetry.update()
        }

        robotState = RobotState.HOMED
        leds.updatePatternForHoming()
        return homingResult
    }

    private fun waitForHome(telemetry: Telemetry) {
        val timer = ElapsedTime()
        timer.reset()
        while (!limitSwitches.isAtHome()) {
            // wait for either limit switch to be activated
            sleep(200)

            // show this whether isTelemetryEnabled is true or not
            telemetry.addData("Waiting for Homing...", timer.seconds())
            telemetry.update()
        }
        telemetry.clearAll()
    }
}

enum class HomingMethod {
    LIMIT_SWITCH, USING_SAVED_FROM_AUTO
}
data class HomingResult(val homeAngles: LimitSwitchHomeAngles, val homePosition: HomePosition, val method: HomingMethod)