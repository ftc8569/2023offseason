package org.firstinspires.ftc.teamcode.commands.wrist

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.trajectory.Trajectory
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator
import com.arcrobotics.ftclib.trajectory.constraint.DifferentialDriveKinematicsConstraint
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Cons.*
import org.firstinspires.ftc.teamcode.subsystems.DiffWrist

class WristTrajectory(var wrist: DiffWrist, var start: Pose2d, var end: Pose2d): CommandBase() {
    var config = TrajectoryConfig(WRIST_MAX_VEL, WRIST_MAX_ACCEL).addConstraint(DifferentialDriveKinematicsConstraint(wrist.kinematics, WRIST_MAX_VEL))
    var interpolatedPose = Pose2d((end.x - start.x) /2, (end.y - start.y)/2, Rotation2d((end.rotation.radians - start.rotation.radians)/2) )
    var trajectory: Trajectory = TrajectoryGenerator.generateTrajectory(listOf(start, interpolatedPose, end), config)
    var controller = RamseteController(WRIST_B, WRIST_ZETA)
    var timer: ElapsedTime = ElapsedTime()

    init {
        timer.reset()
    }

    override fun execute() {
        val sample = trajectory.sample(timer.seconds())
        val speeds = controller.calculate(wrist.odo.pose, sample)

        wrist.speeds = speeds
    }

    override fun isFinished() = timer.seconds() > trajectory.totalTimeSeconds
}