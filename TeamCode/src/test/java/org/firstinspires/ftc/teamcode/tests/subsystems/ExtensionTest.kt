package org.firstinspires.ftc.teamcode.tests.subsystems

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.Extension
import org.firstinspires.ftc.teamcode.utilities.AxonCRServo
import org.junit.Test
import org.mockito.Mockito.mock

class ExtensionTest {

    @Test
    fun length_is_set_properly(){

        val extension = Extension(mock<AxonCRServo>(), mock<Telemetry>())
        extension.length = 0.4

    }
}