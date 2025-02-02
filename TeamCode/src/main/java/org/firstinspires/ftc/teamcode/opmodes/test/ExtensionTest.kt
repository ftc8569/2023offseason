import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.ExtensionLinkageSubsystem
import org.firstinspires.ftc.teamcode.subsystems.Robot


@TeleOp
class ExtensionTest: CommandOpMode() {

    override fun initialize() {
        val robot = Robot(hardwareMap, telemetry)
        val driver = GamepadEx(gamepad1)

        val rightDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
        val leftDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val upDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val downDpad = driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)

        robot.telemetry.addLine("Extension Test Initialized")
        robot.telemetry.update()

        rightDpad.whenPressed(InstantCommand({ robot.extension.targetLength = 0.0}, robot.extension))
        leftDpad.whenPressed(InstantCommand({ robot.extension.targetLength = ExtensionLinkageSubsystem.LOW }, robot.extension))
        upDpad.whenPressed(InstantCommand({ robot.extension.targetLength = ExtensionLinkageSubsystem.MID }, robot.extension))
        downDpad.whenPressed(InstantCommand({ robot.extension.targetLength = ExtensionLinkageSubsystem.HIGH }, robot.extension))
    }

}