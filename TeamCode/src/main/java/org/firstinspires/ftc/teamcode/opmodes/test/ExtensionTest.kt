import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Extension
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo

@TeleOp
class ExtensionTest: LinearOpMode() {

    override fun runOpMode() {
        val scheduler = CommandScheduler.getInstance()
        val dashboard:FtcDashboard = FtcDashboard.getInstance()
        val t = MultipleTelemetry(telemetry, dashboard.telemetry)
        val extension = Extension(AxonCRServo(hardwareMap, "extension", "extension", 500.0, 2500.0))
        val gp1 = GamepadEx(gamepad1)
        waitForStart()

        while(opModeIsActive() && !isStopRequested){
            gp1.readButtons()
            if(gp1.wasJustPressed(GamepadKeys.Button.Y)){
                scheduler.schedule(InstantCommand({extension.length = 0.15}, extension))
            } else if (gp1.wasJustPressed(GamepadKeys.Button.A)){
                scheduler.schedule(InstantCommand({extension.length = 0.075}, extension))
            }

            t.addData("Theta: ", extension.theta)
            t.addData("Power: ", extension.power)
            t.addData("Target Pos", extension.targetPosition)
            t.addData("Current Pos", extension.servo.position)
            t.addData("Inside", extension.inside)
            t.addData("Theta Prime", extension.thetaPrime)
            t.update()
            scheduler.run()
        }
    }

}