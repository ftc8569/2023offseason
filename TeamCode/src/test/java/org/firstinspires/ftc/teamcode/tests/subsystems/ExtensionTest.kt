package org.firstinspires.ftc.teamcode.tests.subsystems

import com.qualcomm.robotcore.hardware.CRServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.Extension
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import org.junit.Test
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.mock


class ExtensionTest {

    @Test
    fun length_is_set_properly(){

        val servo = mock<AxonCRServo>(){
            on {servo} doReturn(mock<CRServoImplEx>())
        }
        val extension = Extension(mock<AxonCRServo>())
        extension.length = 0.4
    }
}