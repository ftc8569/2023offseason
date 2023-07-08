import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.subsystems.Extension
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo


@TeleOp
class ExtensionTest: LinearOpMode() {

    override fun runOpMode() {
//        val scheduler = CommandScheduler.getInstance()
//        val dashboard:FtcDashboard = FtcDashboard.getInstance()
//        val t = MultipleTelemetry(telemetry, dashboard.telemetry)
//        val extension = Extension(hardwareMap.get(ServoImplEx::class.java, "extension"))
//        val gp1 = GamepadEx(gamepad1)
//        waitForStart()
//
//        while(opModeIsActive() && !isStopRequested){
//            gp1.readButtons()
//            if(gp1.wasJustPressed(GamepadKeys.Button.Y)){
//                scheduler.schedule(InstantCommand({extension.length = 0.3}, extension))
//            } else if (gp1.wasJustPressed(GamepadKeys.Button.A)){
//                scheduler.schedule(InstantCommand({extension.home()}, extension))
//            } else if (gp1.wasJustPressed(GamepadKeys.Button.B)){
//                scheduler.schedule(InstantCommand({extension.length=0.4}, extension))
//            }
//
//            t.addData("Phi ", extension.phi)
//            t.addData("Target phi ", extension.targetPhi)
//            t.addData("Theta", extension.theta)
//            t.addData("Target theta", extension.targetTheta)
//            t.addData("Target Pos", extension.targetPosition)
//            t.addData("Current Pos", extension.servo.position)
//            t.update()
//            scheduler.run()
//        }
//        scheduler.reset()
    }

}