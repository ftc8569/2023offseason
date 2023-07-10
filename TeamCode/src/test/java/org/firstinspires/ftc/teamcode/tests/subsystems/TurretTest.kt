package org.firstinspires.ftc.teamcode.tests.subsystems

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import junit.framework.TestCase.assertEquals
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretAngle
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem
import org.junit.Test
import org.mockito.ArgumentMatchers
import org.mockito.kotlin.doAnswer
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.mock

class TurretTest {
    val robot = mock<Robot>()
    private val loopTime = 20e-3

    @Test
    fun motor_acceleration_stubbing_works(){
        val motor = mock<MotorEx>(){
            on {acceleration} doReturn 0.0
        }
        assertEquals(0.0, motor.acceleration)
    }

    @Test
    fun turret_gets_to_position(){
        var curPos = 0.0
        val targetPosition = 1104.21 // 90 degree turn
        var elapsedTime = 0.0

        val motor = mock<MotorEx>(){
            on {acceleration} doReturn 0.0
            on {velocity} doReturn 0.0
            on {set(ArgumentMatchers.anyDouble())} doAnswer {
                curPos += it.getArgument<Double>(0) * loopTime
            }
            on{currentPosition} doAnswer {curPos.toInt()}
        }

        val turret = TurretSubsystem(motor, mock<Robot>(), {0.0})
        turret.robotRelativeTargetAngle = 90.0
        for(i in 0..1500){
            turret.periodic()
        }

        assertEquals(targetPosition, turret.motor.currentPosition.toDouble(), 20.0)

    }

    @Test
    fun turretDoThings(){
        val motor = mock<MotorEx>(){
            on {acceleration} doReturn 0.0
            on {velocity} doReturn 0.0
        }

        val robot = mock<Robot>(){
            on {t} doReturn mock<MultipleTelemetry>()
        }

        val turret = TurretSubsystem(motor, robot) {45.0}
        turret.periodic()
        val angleCommand = SetTurretAngle(turret, -40.0)
        angleCommand.initialize()
        turret.periodic()
        turret.periodic()
    }



}
