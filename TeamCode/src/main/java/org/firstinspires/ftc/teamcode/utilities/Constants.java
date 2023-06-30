package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Elbow Constants
    public static final double ELBOW_KS = 0.0;
    public static final double ELBOW_KA = 0.0;
    public static final double ELBOW_KV = 0.0;
    public static final double ELBOW_KCOS = 0.0;
    public static final double ELBOW_KP = 0.01;
    public static final double ELBOW_KI = 0.0;
    public static final double ELBOW_KD = 0.0;
    public static final double INITIAL_ANGLE = 0.0;

    // Extension Constants
    public static final double LEG_A = 175e-3;
    public static final double LEG_B = 250e-3;
    public static final double EXTENSION_KP = .008;
    public static final double EXTENSION_KI = 0.0;
    public static final double EXTENSION_KD = 0.003;
    public static final double EXTENSION_KCOS = 0.001;

    // Turret Constants
    public static final double TURRET_MAX_V = 10.0;
    public static final double TURRET_MAX_A = 2.0;
    public static final double TURRET_KV = 0.1;
    public static final double TURRET_KA = 0.1;
    public static final double TURRET_KP = 0.007;
    public static final double TURRET_KI = 0.0;
    public static final double TURRET_KD = 0.0000;
    public static final double TURRET_MOTOR_TICKS_PER_REV = 751.8; // For a 223RPM motor
    public static final double MOTOR_TO_TURRET_GEAR_RATIO = 24.0 / 141.0;


    // Claw Constants
    public static final double CLAW_OPEN = 2500;
    public static final double CLAW_CLOSE = 1250;

    // Wrist Constants

    // Starting position constants
    public static final double TURRET_STARTING_ENCODER = 100;
    public static final double EXTENSION_STARTING_ENCODER = 65;
    public static final double ELBOW_EXTENSION_STARTING_ENCODER = 10;

}
