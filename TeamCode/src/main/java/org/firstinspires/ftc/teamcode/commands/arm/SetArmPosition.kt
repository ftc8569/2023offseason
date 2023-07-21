package org.firstinspires.ftc.teamcode.commands.arm

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import org.firstinspires.ftc.teamcode.commands.elbow.SetElbowAngle
import org.firstinspires.ftc.teamcode.commands.extension.SetExtensionLinkage
import org.firstinspires.ftc.teamcode.commands.wrist.SetWristAngles
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.asin
import kotlin.math.atan
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sqrt

class SetArmPosition(robot : Robot, private val getRadius : () -> Double, private val getZHeight : () -> Double, twistAngle : Double = 0.0) : ParallelCommandGroup() {
   constructor(robot: Robot, r: Double, z : Double, twistAngle : Double = 0.0) : this(robot, { r }, { z }, twistAngle)

    private val elbowPivotOffsetFromTurretCenter = 27.6 // mm
    private val elbowPivotHeightAboveGround = 322.0 // mm
    val minimumRadius = 361.0 //mm
    val maximumRadius = minimumRadius + 11.0 * 25.4 // assume we can extend no more than 11 inches
    private val wristPivotOffset = 63.4 // mm
    private val minimumExtensionLength = sqrt(minimumRadius.pow(2.0) - wristPivotOffset.pow(2.0)) // mm

    init {
        val radius = getRadius().coerceIn(minimumRadius, maximumRadius)
        val zHeight = getZHeight().coerceIn(50.0, 600.0)  // for now these are just a reasonable guess
        val wristPivotAngleOffset = -asin(wristPivotOffset/radius)

        addCommands(
            SetElbowAngle(robot.elbow, getElbowAngle(radius, zHeight, wristPivotAngleOffset)),
            SetExtensionLinkage(robot.extension, getExtensionLength(radius, zHeight)),
            SetWristAngles(robot.wrist, getWristBendAngle(radius, zHeight, wristPivotAngleOffset), twistAngle)
        )
    }

    private fun getElbowAngle(r : Double, z : Double, wristPivotAngleOffset : Double) : Double {
        val elbowAngle = wristPivotAngleOffset - atan((elbowPivotHeightAboveGround - z)/(r + elbowPivotOffsetFromTurretCenter))
        return Math.toDegrees(elbowAngle)
    }
    private fun getExtensionLength(r : Double, z : Double) : Double {
        val overallExtensionLength = sqrt((r + elbowPivotOffsetFromTurretCenter).pow(2.0) + (elbowPivotHeightAboveGround - z).pow(2.0))
        return  max(0.0, overallExtensionLength - minimumExtensionLength)
    }
    private fun getWristBendAngle(r : Double, z : Double, wristPivotAngleOffset : Double) : Double {
        val wristBendAngle = (getElbowAngle(r,z, wristPivotAngleOffset) - wristPivotAngleOffset)
        return Math.toDegrees(wristBendAngle)
    }

}