package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase

class RelativeEncoderStateMonitorSubsystem(private val robot: Robot) : SubsystemBase() {
    init {
        register()
    }

    companion object {
        var savedEncoderAngles: SavedEncoderAngles? = null
    }
    var isTelemetryEnabled = false
    var trackEncoderAnglesInAutonomous = true

    override fun periodic() {
        if(robot.opModeType == OpModeType.AUTONOMOUS && trackEncoderAnglesInAutonomous) {
            savedEncoderAngles = SavedEncoderAngles(
                robot.turret.currentAngleDegrees,
                robot.elbow.currentAngleDegrees
            )
        }
        else {
            // this is here for back to back teleop runs.  If we don't clear this, the next teleop run will
            // start with the turret and elbow at the last position they were in at the end of the previous
            // teleop run.
            savedEncoderAngles = null
        }

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Relative Encoder State Monitor: Telemetry Enabled")
            robot.telemetry.addData("OpModeType", robot.opModeType.toString())
            robot.telemetry.addData("Track Encoder Angles In Autonomous", trackEncoderAnglesInAutonomous)
            robot.telemetry.addData("Turret Angle", robot.turret.currentAngleDegrees)
            robot.telemetry.addData("Elbow Angle", robot.elbow.currentAngleDegrees)
            robot.telemetry.update()
        }
    }
}

data class SavedEncoderAngles(val turretAngle : Double, val elbowAngle : Double)