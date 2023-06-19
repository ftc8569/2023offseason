package org.firstinspires.ftc.teamcode.tests.commands

import com.arcrobotics.ftclib.geometry.Vector2d
import org.firstinspires.ftc.teamcode.commands.TurretLock45
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.junit.Test
import org.mockito.kotlin.mock
import kotlin.math.PI

class TurretLock45Tests {

    @Test
    fun some_kinda_test(){
        val turret = mock<Turret>()
        val heading = {PI/2}
        val input = {Vector2d(1.0, 1.0)}
        val command = TurretLock45(turret, heading, input)
        command.execute()
    }


}