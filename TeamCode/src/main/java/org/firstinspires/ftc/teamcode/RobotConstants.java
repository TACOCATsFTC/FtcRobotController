package org.firstinspires.ftc.teamcode;

public final class RobotConstants {
    private RobotConstants() {}

    public static final String FRONT_LEFT_DRIVE  = "front_left_drive";
    public static final String BACK_LEFT_DRIVE   = "back_left_drive";
    public static final String FRONT_RIGHT_DRIVE = "front_right_drive";
    public static final String BACK_RIGHT_DRIVE  = "back_right_drive";
    public static final String INTAKE           = "intake";

    public static final double DEFAULT_MAX_WHEEL_SPEED = 0.75;
    public static final double SLOW_WHEEL_SPEED   = 0.25;
    public static final double MEDIUM_WHEEL_SPEED = 0.5;
    public static final double FAST_WHEEL_SPEED   = 0.75;
    public static final double TURBO_WHEEL_SPEED  = 1.0;
    public static final double JOYSTICK_DEADBAND  = 0.05;

    public static final double DEFAULT_INTAKE_SPEED = 1;
    public static final double INTAKE_SPEED_STEP   = 0.001;

    public static final String OTOS_NAME = "sensor_otos";

    // Measure these from the robot center to the OTOS center.
    public static final double OTOS_OFFSET_X_IN = 0.0;
    public static final double OTOS_OFFSET_Y_IN = 0.0;
    public static final double OTOS_OFFSET_HEADING_DEG = 0.0;

    // Replace after calibration. Valid range: 0.872 through 1.127.
    public static final double OTOS_LINEAR_SCALAR = 1.0;
    public static final double OTOS_ANGULAR_SCALAR = 1.0;

    public static final double POSITION_TOLERANCE_IN = 1.0;
    public static final double HEADING_TOLERANCE_DEG = 3.0;
    public static final double POSITION_SLOWDOWN_IN = 12.0;
    public static final double HEADING_SLOWDOWN_DEG = 45.0;
    public static final double MIN_DRIVE_POWER = 0.12;
    public static final double MIN_TURN_POWER = 0.10;
    public static final double WAYPOINT_TIMEOUT_SECONDS = 8.0;
}
