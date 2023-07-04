package org.firstinspires.ftc.teamcode.tests.commands

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics
import com.arcrobotics.ftclib.trajectory.Trajectory
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator
import com.arcrobotics.ftclib.trajectory.constraint.DifferentialDriveKinematicsConstraint
import org.firstinspires.ftc.teamcode.Cons
import org.junit.Test
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


}