package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.Cons

data class ArmStateData(
    val wrist: WristStateData,
    val elbow: ElbowStateData,
    val extension: ExtensionStateData,
    val aligner: PoleAlignerStateData,
    val claw: ClawStateData)
data class ArmAndTurretStateData(
    val arm: ArmStateData,
    val turret: TurretStateData)
data class TurretStateData(
    val angle: Double)
data class WristStateData(
    val bendAngle: Double,
    val twistAngle: Double,
    val depositBendAngle : Double = 0.0,
    val depositTwistAngle : Double = 0.0)

data class ElbowStateData(
    val angle: Double)

data class ExtensionStateData(
    val length: Double)

data class PoleAlignerStateData(
    val     angle: Double)

data class ClawStateData(
    val angle : Double)
public class ArmStatePositionData() {
    companion object {
        val CLAW_OPEN_FOR_INTAKE = ClawStateData((Cons.CLAW_OPEN_FOR_INTAKE - 1500.0)/2000.0 * 180.0)
        val CLAW_HOLD_CONE = ClawStateData((Cons.CLAW_HOLD_CONE - 1500.0)/2000.0 * 180.0)
        val CLAW_RELEASE_CONE_BUT_HOLD_TSE = ClawStateData((Cons.CLAW_RELEASE_CONE_BUT_HOLD_TSE - 1500.0)/2000.0 * 180.0)

        val ARM_HOME = ArmStateData(
            WristStateData(-113.0, 0.0),
            ElbowStateData(0.0),
            ExtensionStateData(0.0),
            PoleAlignerStateData(-149.0),
            CLAW_OPEN_FOR_INTAKE
        )
        val ARM_CONE_RIGHTING = ArmStateData(
                WristStateData(-33.0, 0.0),
                ElbowStateData(-18.0),
                ExtensionStateData(4.75),
                PoleAlignerStateData(-22.0),
                CLAW_OPEN_FOR_INTAKE
        )
        val ARM_FALLEN_CONE_HOVER = ArmStateData(
                WristStateData(80.0, 0.0),
                ElbowStateData(-25.0),
                ExtensionStateData(4.0),
                ARM_HOME.aligner,
                CLAW_OPEN_FOR_INTAKE
        )
        val START_LEFT = ArmAndTurretStateData(
            ArmStateData(
                ARM_HOME.wrist,
                ElbowStateData(-45.0),
                ARM_HOME.extension,
                ARM_HOME.aligner,
                CLAW_HOLD_CONE
            ),
            TurretStateData(-53.0)
        )
        val START_RIGHT = ArmAndTurretStateData(
            ArmStateData(
                ARM_HOME.wrist,
                ElbowStateData(-50.5),
                ARM_HOME.extension,
                ARM_HOME.aligner,
                CLAW_HOLD_CONE
            ),
            TurretStateData(40.0)
        )
        val SCORE_HIGH = ArmStateData(
            WristStateData(-5.0, ARM_HOME.wrist.twistAngle, 59.0),
            ElbowStateData(58.0),
            ExtensionStateData(9.5),
            PoleAlignerStateData(-35.0),
            CLAW_HOLD_CONE
        )
        val SCORE_MEDIUM = ArmStateData(
            WristStateData(2.0, ARM_HOME.wrist.twistAngle, 38.0),
            ElbowStateData(30.0),
            ExtensionStateData(0.0),
            PoleAlignerStateData(-11.0),
            CLAW_HOLD_CONE
        )
        val SCORE_LOW = ArmStateData(
            WristStateData(-45.0, ARM_HOME.wrist.twistAngle, -10.0),
            ElbowStateData(-20.0),
            ExtensionStateData(0.0),
            ARM_HOME.aligner,
            CLAW_HOLD_CONE
        )
        val TILT_POLE = ArmStateData(
                WristStateData(-70.0, ARM_HOME.wrist.twistAngle),
                ElbowStateData(10.0),
                ExtensionStateData(10.0),
                PoleAlignerStateData(-10.0),
                CLAW_OPEN_FOR_INTAKE
        )
        val SCORE_LOW_UPSIDEDOWN = ArmStateData(
            WristStateData(-10.0, 180.0, 7.0),
            ElbowStateData(3.0),
            ExtensionStateData(0.0),
            ARM_HOME.aligner,
            CLAW_HOLD_CONE
        )
        var INTAKE = ArmStateData(
            WristStateData(-25.0, ARM_HOME.wrist.twistAngle),
            ElbowStateData(-45.0),
            ExtensionStateData(6.5),
            PoleAlignerStateData(ARM_HOME.aligner.angle),
            CLAW_OPEN_FOR_INTAKE
        )
        val SCORE_GROUND = ArmStateData(
            WristStateData(INTAKE.wrist.bendAngle, INTAKE.wrist.twistAngle, INTAKE.wrist.bendAngle),
            INTAKE.elbow,
            ExtensionStateData(INTAKE.extension.length - 1.0),
            PoleAlignerStateData(ARM_HOME.aligner.angle),
            CLAW_HOLD_CONE
        )
        val TRAVEL = ArmStateData(
            WristStateData(-110.0, ARM_HOME.wrist.twistAngle),
            ElbowStateData(45.0),
            ExtensionStateData(0.0),
            PoleAlignerStateData(ARM_HOME.aligner.angle),
            CLAW_HOLD_CONE
        )
        val HIGH_POLE_TO_CONESTACK = ArmAndTurretStateData(
            ArmStateData(
                WristStateData(-110.0, 0.0),
                ElbowStateData(45.0),
                ExtensionStateData(8.0),
                PoleAlignerStateData(ARM_HOME.aligner.angle),
                CLAW_HOLD_CONE
            ),
            TurretStateData(92.0)
        )

        val CONESTACK_TO_HIGH_POLE = ArmAndTurretStateData(
            ArmStateData(
                WristStateData(-110.0, 0.0),
                ElbowStateData(45.0),
                ExtensionStateData(8.0),
                PoleAlignerStateData(ARM_HOME.aligner.angle),
                CLAW_HOLD_CONE
            ),
            TurretStateData(-53.2)
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