package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Drives through field-relative waypoints using a SparkFun OTOS.
 * Coordinates start at the robot's initial pose: +X right, +Y forward,
 * and positive heading counterclockwise.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Yoshi OTOS Auto", group = "Robot")
public class Autonomous extends LinearOpMode {
    private static final String OTOS_NAME = "sensor_otos";

    // Measure these from the robot center to the OTOS center.
    private static final double OTOS_OFFSET_X_IN = 0.0;
    private static final double OTOS_OFFSET_Y_IN = 0.0;
    private static final double OTOS_OFFSET_HEADING_DEG = 0.0;

    // Replace after calibration. Valid range: 0.872 through 1.127.
    private static final double OTOS_LINEAR_SCALAR = 1.0;
    private static final double OTOS_ANGULAR_SCALAR = 1.0;

    private static final double POSITION_TOLERANCE_IN = 1.0;
    private static final double HEADING_TOLERANCE_DEG = 3.0;
    private static final double POSITION_SLOWDOWN_IN = 12.0;
    private static final double HEADING_SLOWDOWN_DEG = 45.0;
    private static final double MIN_DRIVE_POWER = 0.12;
    private static final double MIN_TURN_POWER = 0.10;
    private static final double WAYPOINT_TIMEOUT_SECONDS = 8.0;

    // Each point is {X inches, Y inches, heading degrees, maximum power}.
    private static final double[][] WAYPOINTS = {
            { 0.0, 24.0,   0.0, 0.50},
            {24.0, 24.0,  90.0, 0.45},
            {24.0,  0.0, 180.0, 0.45},
            { 0.0,  0.0,   0.0, 0.40}
    };

    private final Yoshi robot = new Yoshi();
    private final ElapsedTime waypointTimer = new ElapsedTime();
    private SparkFunOTOS otos;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        otos = hardwareMap.get(SparkFunOTOS.class, OTOS_NAME);
        configureOtos();

        telemetry.addLine("OTOS ready");
        telemetry.addLine("Keep the robot still, then press Start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));

        for (int i = 0; i < WAYPOINTS.length && opModeIsActive(); i++) {
            double[] point = WAYPOINTS[i];
            boolean reached = driveToPoint(i + 1, point[0], point[1], point[2], point[3]);
            if (!reached) {
                telemetry.addData("Warning", "Waypoint %d timed out", i + 1);
                telemetry.update();
                break;
            }
        }

        robot.stopDrive();
        SparkFunOTOS.Pose2D finalPose = otos.getPosition();
        telemetry.addLine("Autonomous complete");
        telemetry.addData("Final pose", "X %.1f  Y %.1f  H %.1f",
                finalPose.x, finalPose.y, finalPose.h);
        telemetry.update();
        sleep(1000);
    }

    private void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(
                OTOS_OFFSET_X_IN, OTOS_OFFSET_Y_IN, OTOS_OFFSET_HEADING_DEG));
        otos.setLinearScalar(OTOS_LINEAR_SCALAR);
        otos.setAngularScalar(OTOS_ANGULAR_SCALAR);

        telemetry.addLine("Calibrating OTOS IMU - do not move the robot");
        telemetry.update();
        otos.calibrateImu();
        otos.resetTracking();
    }

    private boolean driveToPoint(int pointNumber, double targetX, double targetY,
                                 double targetHeading, double maxPower) {
        waypointTimer.reset();

        while (opModeIsActive() && waypointTimer.seconds() < WAYPOINT_TIMEOUT_SECONDS) {
            SparkFunOTOS.Pose2D pose = otos.getPosition();
            double errorX = targetX - pose.x;
            double errorY = targetY - pose.y;
            double headingError = normalizeDegrees(targetHeading - pose.h);
            double distance = Math.hypot(errorX, errorY);

            boolean positionReached = distance <= POSITION_TOLERANCE_IN;
            boolean headingReached = Math.abs(headingError) <= HEADING_TOLERANCE_DEG;
            if (positionReached && headingReached) {
                robot.stopDrive();
                return true;
            }

            double headingRadians = Math.toRadians(pose.h);
            double robotAxialError = errorY * Math.cos(headingRadians)
                    - errorX * Math.sin(headingRadians);
            double robotLateralError = errorX * Math.cos(headingRadians)
                    + errorY * Math.sin(headingRadians);

            double axialPower = 0.0;
            double lateralPower = 0.0;
            if (!positionReached) {
                double translationPower = proportionalPower(
                        distance, POSITION_SLOWDOWN_IN, MIN_DRIVE_POWER, maxPower);
                axialPower = robotAxialError / distance * translationPower;
                lateralPower = robotLateralError / distance * translationPower;
            }

            double yawPower = 0.0;
            if (!headingReached) {
                double turnLimit = Math.min(maxPower, 0.40);
                double turnPower = proportionalPower(Math.abs(headingError),
                        HEADING_SLOWDOWN_DEG, MIN_TURN_POWER, turnLimit);
                // Yoshi.drive() uses negative yaw for a positive OTOS turn.
                yawPower = -Math.signum(headingError) * turnPower;
            }

            robot.drive(axialPower, lateralPower, yawPower, 1.0);
            telemetry.addData("Waypoint", "%d / %d", pointNumber, WAYPOINTS.length);
            telemetry.addData("Target", "X %.1f  Y %.1f  H %.1f",
                    targetX, targetY, targetHeading);
            telemetry.addData("Position", "X %.1f  Y %.1f  H %.1f",
                    pose.x, pose.y, pose.h);
            telemetry.addData("Error", "Distance %.1f in  Heading %.1f deg",
                    distance, headingError);
            telemetry.addData("Drive", "Axial %.2f  Lateral %.2f  Yaw %.2f",
                    axialPower, lateralPower, yawPower);
            telemetry.update();
            idle();
        }

        robot.stopDrive();
        return false;
    }

    private double proportionalPower(double error, double slowdownRange,
                                     double minimum, double maximum) {
        double scaled = maximum * Math.min(1.0, error / slowdownRange);
        return Math.min(maximum, Math.max(minimum, scaled));
    }

    private double normalizeDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees -= 360.0;
        }
        while (degrees <= -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }
}
