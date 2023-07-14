package org.firstinspires.ftc.teamcode.commands.general

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.pow

class EasingManager(private var easingDuration : Double, private var easingStart :Double, private var easingEnd : Double) {
    private val easingTimer = ElapsedTime()

    fun startEasing() {
        easingTimer.reset()
    }
    fun seconds() : Double {
        return easingTimer.seconds()
    }
    fun milliseconds() : Double {
        return easingTimer.milliseconds()
    }
    fun getEasingValue(): Double {
        return if (seconds() > easingDuration) easingEnd
        else if (seconds() < 0.0) easingStart
        else {
            val t = easingTimer.seconds()
            var time = t / (easingDuration / 2.0)
            return if (time < 1.0) {
                (easingEnd - easingStart) / 2.0 * time.pow(2.0) + easingStart
            } else {
                -1 * (easingEnd - easingStart) / 2.0 * (--time * (time - 2.0) - 1.0) + easingStart
            }
        }
    }
    fun isFinished(): Boolean {
        return easingTimer.seconds() > easingDuration
    }
}