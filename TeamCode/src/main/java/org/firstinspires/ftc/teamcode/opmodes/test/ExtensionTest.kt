import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Extension
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo

@TeleOp
class ExtensionTest: CommandOpMode() {
    override fun initialize() {
        val dashboard:FtcDashboard = FtcDashboard.getInstance()
        val extension = Extension(AxonCRServo(hardwareMap, "extension", "extension", 500.0, 2500.0))
        val gp1 = GamepadEx(gamepad1)
        val rightDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
        val leftDpad = gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val upDpad = gp1.getGamepadButton(GamepadKeys.Button.Y)
        val downDpad = gp1.getGamepadButton(GamepadKeys.Button.A)

        upDpad.whenPressed(InstantCommand({extension.length = 0.3}, extension))
        downDpad.whenPressed(InstantCommand({extension.length = 0.0}, extension))
    }

}