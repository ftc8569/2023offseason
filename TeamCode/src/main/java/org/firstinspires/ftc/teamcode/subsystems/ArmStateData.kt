package org.firstinspires.ftc.teamcode.subsystems

data class ArmStateData(
    val wrist: WristStateData,
    val elbow: ElbowStateData,
    val extension: ExtensionStateData,
    val aligner: PoleAlignerStateData)
data class ArmAndTurretStateData(
    val arm: ArmStateData,
    val turret: TurretStateData)
data class TurretStateData(
    val angle: Double)
data class WristStateData(
    val bendAngle: Double,
    val twistAngle: Double,
    val depositBendAngle : Double = 0.0)

data class ElbowStateData(
    val angle: Double)

data class ExtensionStateData(
    val length: Double)

data class PoleAlignerStateData(
    val angle: Double)

public class ArmStates() {
    companion object {
        val ARM_HOME = ArmStateData(
            WristStateData(-92.0, 0.0),
            ElbowStateData(-45.0),
            ExtensionStateData(0.0),
            PoleAlignerStateData(-75.0)
        )
        val START_LEFT = ArmAndTurretStateData(
            ArmStateData(
                ARM_HOME.wrist,
                ElbowStateData(-45.0),
                ARM_HOME.extension,
                ARM_HOME.aligner
            ),
            TurretStateData(-53.0)
        )
        val START_RIGHT = ArmAndTurretStateData(
            ArmStateData(
                ARM_HOME.wrist,
                ElbowStateData(-50.5),
                ARM_HOME.extension,
                ARM_HOME.aligner
            ),
            TurretStateData(55.0)
        )
        val SCORE_HIGH = ArmStateData(
            WristStateData(-2.0, 0.0, 34.0),
            ElbowStateData(60.0),
            ExtensionStateData(14.5),
            PoleAlignerStateData(8.0)
        )
        val SCORE_MIDDLE = ArmStateData(
            WristStateData(-12.0, 0.0, 27.0),
            ElbowStateData(35.0),
            ExtensionStateData(0.0),
            PoleAlignerStateData(39.0)
        )
        val SCORE_LOW = ArmStateData(
            WristStateData(-10.0, 0.0, 7.0),
            ElbowStateData(3.0),
            ExtensionStateData(0.0),
            PoleAlignerStateData(50.0)
        )
        val INTAKE = ArmStateData(
            WristStateData(-37.0, 0.0),
            ElbowStateData(-49.0),
            ExtensionStateData(7.25),
            PoleAlignerStateData(ARM_HOME.aligner.angle)
        )
        val SCORE_GROUND = ArmStateData(
            WristStateData(-37.0, 0.0, -37.0),
            ElbowStateData(-49.0),
            ExtensionStateData(6.5),
            PoleAlignerStateData(ARM_HOME.aligner.angle)
        )
        val TRAVEL = ArmStateData(
            WristStateData(-92.0, 0.0),
            ElbowStateData(-26.0),
            ExtensionStateData(0.0),
            PoleAlignerStateData(ARM_HOME.aligner.angle)
        )
        val INTAKE_REAR = ArmAndTurretStateData(
            INTAKE,
            TurretStateData(180.0)
        )
        val INTAKE_FRONT = ArmAndTurretStateData(
            INTAKE,
            TurretStateData(0.0)
        )
        val INTAKE_RIGHT = ArmAndTurretStateData(
            INTAKE,
            TurretStateData(90.0)
        )
        val INTAKE_LEFT = ArmAndTurretStateData(
            INTAKE,
            TurretStateData(-90.0)
        )
    }
}