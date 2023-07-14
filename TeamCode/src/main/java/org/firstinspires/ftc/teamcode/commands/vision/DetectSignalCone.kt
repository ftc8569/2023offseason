package org.firstinspires.ftc.teamcode.commands.vision

import android.util.Size
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.apriltags.ConeNumber
import org.firstinspires.ftc.teamcode.subsystems.Robot

class DetectSignalCone(val robot : Robot) : CommandBase() {
    init {
        addRequirements(robot.signalSleeve)
    }
    override fun initialize() {
        robot.signalSleeve.start(Size( 1920, 1080))
    }
 override fun execute() {
     robot.detectedSignalCone = robot.signalSleeve.getSignalConeDetected()
    }
    override fun isFinished(): Boolean {
        return if(robot.detectedSignalCone != ConeNumber.NONE) {
            robot.signalSleeve.shutdown()
            true
        } else false
    }
}