package org.firstinspires.ftc.teamcode;

/**
 * Single place for every tunable number and device name.
 * When the robot changes between seasons, start here.
 */
public final class RobotConstants {
    private RobotConstants() {}

    // Hardware configuration names (must match the Robot Controller config)
    public static final String FRONT_LEFT_DRIVE  = "front_left_drive";
    public static final String BACK_LEFT_DRIVE   = "back_left_drive";
    public static final String FRONT_RIGHT_DRIVE = "front_right_drive";
    public static final String BACK_RIGHT_DRIVE  = "back_right_drive";
    public static final String SHOOTER           = "Shooter";
    public static final String FRONT_LED         = "frontled";
    public static final String TRIGGER_1         = "trigger1";
    public static final String TRIGGER_2         = "trigger2";

    // Odometry (GoBilda Pinpoint — must match Robot Controller config)
    public static final String PINPOINT          = "pinpoint";
    // Distance of each pod from the robot center in mm.
    // Positive X = in front of center, positive Y = left of center.
    // Measure on your robot and update these before competition.
    public static final double ODO_X_OFFSET_MM   = 0.0;
    public static final double ODO_Y_OFFSET_MM   = 0.0;

    // Drive
    public static final double DEFAULT_MAX_WHEEL_SPEED = 0.75;
    public static final double SLOW_WHEEL_SPEED   = 0.25;
    public static final double MEDIUM_WHEEL_SPEED = 0.5;
    public static final double FAST_WHEEL_SPEED   = 0.75;
    public static final double TURBO_WHEEL_SPEED  = 1.0;
    public static final double JOYSTICK_DEADBAND  = 0.05;

    // Shooter
    public static final double DEFAULT_SHOOTER_SPEED = 0.55;
    public static final double SHOOTER_SPEED_STEP   = 0.001;

    // Front LED servo positions
    public static final double LED_POSITION_A = 0.0;
    public static final double LED_POSITION_B = 1.0;

    // Autonomous
    public static final double AUTO_FORWARD_SPEED = 0.6;
    public static final double AUTO_TURN_SPEED    = 0.5;
}
