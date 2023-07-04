package org.firstinspires.ftc.teamcode.tests.commands

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics
import com.arcrobotics.ftclib.trajectory.Trajectory
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator
import com.arcrobotics.ftclib.trajectory.constraint.DifferentialDriveKinematicsConstraint
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServoImplEx
import junit.framework.TestCase.assertEquals
import org.firstinspires.ftc.teamcode.Cons
import org.firstinspires.ftc.teamcode.subsystems.DiffWrist
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import org.firstinspires.ftc.teamcode.utilities.RollingAverageFilter
import org.junit.Test
import org.mockito.kotlin.doAnswer
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.mock
import kotlin.math.PI

class WristTrajectoryTest {

    @Test
    fun figure_it_the_fuck_out() {
        val kinematics = DifferentialDriveKinematics(0.0322)
        var config = TrajectoryConfig(Cons.WRIST_MAX_VEL, Cons.WRIST_MAX_ACCEL).addConstraint(DifferentialDriveKinematicsConstraint(kinematics, 0.1))
        var trajectory: Trajectory = TrajectoryGenerator.generateTrajectory(
            listOf(
                Pose2d(0.0, 0.0, Rotation2d(0.0)), Pose2d(
                    0.0001, 0.0001, Rotation2d(
                        PI
                    )
                )
            ), config
        )
        println(trajectory.totalTimeSeconds)
        println(trajectory)
    }

    @Test
    fun no_movement_test(){
        val odo = DifferentialOdometry({0.0}, {0.0}, 1.5)
        for(i in 0..10){
            odo.updatePose()
        }

        assertEquals(0.0, odo.pose.rotation.degrees, 0.001)
    }
    @Test
    fun straight_line_test(){
        val odo = DifferentialOdometry({1.0}, {1.0}, 1.5)
        odo.updatePose()
        assertEquals(1.0, odo.pose.translation.x, 0.01)
        assertEquals(0.0, odo.pose.rotation.degrees, 0.01)
    }

    @Test
    fun wrist_test(){
        var numEncoderCalls = 0
        val leftServo = mock<AxonCRServo>(){
            on {position} doAnswer{
                var out = 0.0
                when(numEncoderCalls){
                    0 -> out = 0.0
                    1 -> out = 100.0
                    2 -> out = 100.0
                    else -> out = 100.0
                }
                out
            }
            on {velocity} doReturn mock<RollingAverageFilter>()
            on {velocity.get()} doAnswer {
                var out = 0.0
                when(numEncoderCalls) {
                    0 -> out = 0.0
                    1 -> out = 100.0
                    2 -> out = 100.0
                    else -> out = 100.0
                }
                out
            }

        }
        val rightServo = mock<AxonCRServo>(){
            on {position} doAnswer{
                var out = 0.0
                when(numEncoderCalls){
                    0 -> out = 0.0
                    1 -> out = -100.0
                    2 -> out = -100.0
                    else -> out = -100.0
                }
                numEncoderCalls++
                out
            }
            on {velocity} doReturn mock<RollingAverageFilter>()
            on {velocity.get()} doAnswer {
                var out = 0.0
                when(numEncoderCalls) {
                    0 -> out = 0.0
                    1 -> out = -100.0
                    2 -> out = -100.0
                    else -> out =-100.0
                }
                out
            }
        }

        val wrist = DiffWrist(leftServo, rightServo, mock<MultipleTelemetry>())
        wrist.periodic()
        wrist.periodic()
        wrist.periodic()

    }


}