package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Cons {
    // Elbow Constants
    public static double ELBOW_KS = 0.0;
    public static double ELBOW_KA = 0.0;
    public static double ELBOW_KV = 0.0;
    public static double ELBOW_KCOS = 0.0;
    public static double ELBOW_KP = 0.018;
    public static double ELBOW_KI = 0.05;
    public static double ELBOW_KD = 0.0015;
    public static double INITIAL_ANGLE = 0.0;

    // Extension Constants
    public static double LEG_A = 175e-3;
    public static double LEG_B = 250e-3;
    public static double EXTENSION_KP = .03;
    public static double EXTENSION_KI = 0.0;
    public static double EXTENSION_KD = 5.0;
    public static double EXTENSION_KCOS = 0.001;

    // Turret Constants
    public static double TURRET_MAX_V = 10.0;
    public static double TURRET_MAX_A = 2.0;
    public static double TURRET_KV = 0.1;
    public static double TURRET_KA = 0.1;
    public static double TURRET_KP = 0.007;
    public static double TURRET_KI = 0.0;
    public static double TURRET_KD = 0.0000;
    public static double TURRET_MOTOR_TICKS_PER_REV = 751.8; // For a 223RPM motor
    public static double MOTOR_TO_TURRET_GEAR_RATIO = 24.0 / 141.0;


    // Claw Constants
    public static double CLAW_OPEN = 2500;
    public static double CLAW_CLOSE = 1250;

    // Wrist Constants
    public static double WRIST_LEFT_KP = 0.0005;
    public static double WRIST_LEFT_KD = 0.0001;
    public static double WRIST_RIGHT_KP = 0.0005;
    public static double WRIST_RIGHT_KD = 0.0001;
    public static double WRIST_MAX_VEL = 0.125;
    public static double WRIST_MAX_ACCEL = 0.1;
    public static double WRIST_B = 2.0;
    public static double WRIST_ZETA = 0.7;

    // Aligner Constants
    public static double ALIGNER_IN = 2200;
    public static double ALIGNER_OUT = 826;

    // Starting position constants
    public static double TURRET_STARTING_ENCODER = 100;
    public static double EXTENSION_STARTING_ENCODER = 65;
    public static double ELBOW_EXTENSION_STARTING_ENCODER = 10;

}
