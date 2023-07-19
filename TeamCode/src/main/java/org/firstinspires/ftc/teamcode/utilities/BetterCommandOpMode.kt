package org.firstinspires.ftc.teamcode.utilities

import com.arcrobotics.ftclib.command.*
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

abstract class BetterCommandOpMode:  LinearOpMode(){

    /**
     * Cancels all previous commands
     */
    fun reset() {
        CommandScheduler.getInstance().reset()
    }

    /**
     * Runs the [CommandScheduler] instance
     */
    fun run() {
        CommandScheduler.getInstance().run()
    }

    /**
     * Schedules [com.arcrobotics.ftclib.command.Command] objects to the scheduler
     */
    fun schedule(vararg commands: Command?) {
        CommandScheduler.getInstance().schedule(*commands)
    }

    /**
     * Registers [com.arcrobotics.ftclib.command.Subsystem] objects to the scheduler
     */
    fun register(vararg subsystems: Subsystem?) {
        CommandScheduler.getInstance().registerSubsystem(*subsystems)
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize()
        waitForStart()

        // run the scheduler
        while (!isStopRequested && opModeIsActive()) {
            run()
        }
        reset()
    }

    abstract fun preinit()

    abstract fun initialize()

    fun disable() {
        Robot.disable()
    }

    fun enable() {
        Robot.enable()
    }


}