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
    public static double EXTENSION_KP = .01;
    public static double EXTENSION_KI = 0.0;
    public static double EXTENSION_KD = 0.001;
    public static double EXTENSION_KCOS = 0.001;

    // Turret Constants
    public static double TURRET_MAX_V = 1500;
    public static double TURRET_MAX_A = 800;
    public static double TURRET_KV = 0.0005;
    public static double TURRET_KA = 0.0001;
    public static double TURRET_KP = 2;
    public static double TURRET_KI = 0.0;
    public static double TURRET_KD = 0.0002;
    public static double TURRET_MOTOR_TICKS_PER_REV = 751.8; // For a 223RPM motor
    public static double MOTOR_TO_TURRET_GEAR_RATIO = 24.0 / 141.0;


    // Claw Constants
    public static double CLAW_OPEN = 1500;
    public static double CLAW_CLOSE = 1100;

    // Wrist Constants
    public static double WRIST_LEFT_KP = 0.0005;
    public static double WRIST_LEFT_KD = 0.0001;
    public static double WRIST_RIGHT_KP = 0.0005;
    public static double WRIST_RIGHT_KD = 0.0001;
    public static double WRIST_MAX_VEL = 0.125;
    public static double WRIST_MAX_ACCEL = 0.1;
    public static double WRIST_B = 2.0;
    public static double WRIST_ZETA = 0.7;

    // Extension
    public static double EXTENSION_HOME = 0.6;
    public static double EXTENSION_MAX = 0.0;


    // Starting position constants
    public static double TURRET_MAX_ANGULAR_VELOCITY = 360.0;
    public static double TURRET_MAX_ANGULAR_ACCELERATION = 720.0;


    // Drivetrain Constants
    public static double HEADING_KP = 5;
    public static double HEADING_KD = 1;

}
