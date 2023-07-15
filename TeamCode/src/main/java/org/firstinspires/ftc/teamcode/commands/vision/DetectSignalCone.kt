package org.firstinspires.ftc.teamcode.commands.vision

import android.util.Size
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.apriltags.ConeNumber
import org.firstinspires.ftc.teamcode.subsystems.Robot

class DetectSignalCone(val robot : Robot) : CommandBase() {
    val timer = ElapsedTime()
    init {
        addRequirements(robot.signalSleeve)
    }
    override fun initialize() {
        robot.signalSleeve.start(Size( 1920, 1080))
        timer.reset()
    }
 override fun execute() {
     robot.detectedSignalCone = robot.signalSleeve.getSignalConeDetected()
    }
    override fun isFinished(): Boolean {
        return if(robot.detectedSignalCone != ConeNumber.NONE || timer.seconds() > 10.0) {
            robot.detectedSignalCone = ConeNumber.ONE
            robot.signalSleeve.shutdown()
            true
        } else false
    }
}