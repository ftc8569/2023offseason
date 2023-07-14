package org.firstinspires.ftc.teamcode.commands.extension

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.commands.general.EasingManager
import org.firstinspires.ftc.teamcode.subsystems.ExtensionLinkageSubsystem
import kotlin.math.pow

class SetExtensionLinkage(private val linkage : ExtensionLinkageSubsystem, private val length_in : Double, val easingDuration : Double = 0.0) : CommandBase() {

    val timeoutTimer = ElapsedTime()
    val easingManager = EasingManager(easingDuration, linkage.targetLength, length_in)
    val isEasing = easingDuration > 0.0
    private var easingStartLength = 0.0
    private var easingEndLength = 0.0

    init {
        addRequirements(linkage)
    }

    override fun initialize() {
        if(!isEasing) {
            linkage.targetLength = length_in
            println("SetExtensionLinkage (not easing): distance " + linkage.targetLength)
        } else {
            timeoutTimer.reset()
            easingManager.startEasing()
        }
    }
    override fun execute() {
        if(isEasing) {
            val easedLength = easingManager.getEasingValue()
            linkage.targetLength = easedLength
        }
    }
    override fun isFinished(): Boolean {
        return if (isEasing)
            easingManager.isFinished() || linkage.targetLength == length_in
        else
            linkage.isCloseEnoughToTargetLength() || timeoutTimer.milliseconds() > 500
    }

}