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
    public static final String INTAKE           = "intake";

    // Drive
    public static final double DEFAULT_MAX_WHEEL_SPEED = 0.75;
    public static final double SLOW_WHEEL_SPEED   = 0.25;
    public static final double MEDIUM_WHEEL_SPEED = 0.5;
    public static final double FAST_WHEEL_SPEED   = 0.75;
    public static final double TURBO_WHEEL_SPEED  = 1.0;
    public static final double JOYSTICK_DEADBAND  = 0.05;

    // bugbug: these aren't shooter variables
    // Shooter
    // bugbug: is this correct? Shouldn't it be a percentage?
    public static final double DEFAULT_INTAKE_SPEED = 55; 
    public static final double INTAKE_SPEED_STEP   = 0.001;


}
