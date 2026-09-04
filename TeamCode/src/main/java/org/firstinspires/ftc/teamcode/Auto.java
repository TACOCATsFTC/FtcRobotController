package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Drives through field-relative waypoints using a SparkFun OTOS.
 * Coordinates start at the robot's initial pose: +X right, +Y forward,
 * and positive heading counterclockwise.
 */
@Autonomous(name = "Yoshi OTOS Auto", group = "Robot")
public class Auto extends LinearOpMode {
    private static final String TAG = "TacoCat";

    private final Yoshi robot = new Yoshi();
    private final ElapsedTime waypointTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized. Updated 9/2/26 at 12:11pm");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        resetOtosPosition();


        //edit to cahnge the path of the robot, the first two numbers are the x and y coordinates, and the third number is the heading in degrees
        driveToPosition(0.0, 24.0, 0.0, 1.00);
        sleep(2500);
        driveToPosition(24.0, 24.0, 90.0, 1.00);
        //sleep(2500);
        //driveToPosition(24.0, 0.0, 180.0, 0.45);
        //sleep(2500);
        //driveToPosition(0.0, 0.0, 0.0, 0.40);

        stopAuto();
    }

    //dont touch this code, it is very fragile and will break the robot if you do
    private void resetOtosPosition() {
        robot.otos.resetTracking();
        robot.otos.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
    }

    private void stopAuto() {
        robot.stopDrive();
        SparkFunOTOS.Pose2D finalPose = robot.otos.getPosition();
        Log.d(TAG, String.format("stopAuto: finalPose X=%.2f Y=%.2f H=%.2f",
                finalPose.x, finalPose.y, finalPose.h));
        telemetry.addLine("Autonomous complete");
        telemetry.addData("Final pose", "X %.1f  Y %.1f  H %.1f",
                finalPose.x, finalPose.y, finalPose.h);
        telemetry.update();
        sleep(1000);
    }

    private void driveToPosition(double targetX, double targetY,
                                 double targetHeading, double maxPower) {
        waypointTimer.reset();
        Log.d(TAG, String.format("driveToPosition START target X=%.2f Y=%.2f H=%.2f maxPower=%.2f",
                targetX, targetY, targetHeading, maxPower));

        while (opModeIsActive() && waypointTimer.seconds() < RobotConstants.WAYPOINT_TIMEOUT_SECONDS) {
            SparkFunOTOS.Pose2D pose = robot.otos.getPosition();
            double errorX = targetX - pose.x;
            double errorY = targetY - pose.y;
            double headingError = normalizeDegrees(targetHeading - pose.h);
            double distance = Math.hypot(errorX, errorY);

            boolean positionReached = distance <= RobotConstants.POSITION_TOLERANCE_IN;
            boolean headingReached = Math.abs(headingError) <= RobotConstants.HEADING_TOLERANCE_DEG;
            if (positionReached && headingReached) {
                Log.d(TAG, String.format("driveToPosition REACHED t=%.2fs pose X=%.2f Y=%.2f H=%.2f",
                        waypointTimer.seconds(), pose.x, pose.y, pose.h));
                robot.stopDrive();
                return;
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
                        distance, RobotConstants.POSITION_SLOWDOWN_IN, RobotConstants.MIN_DRIVE_POWER, maxPower);
                // Yoshi.drive() uses negative axial for a positive OTOS forward movement.
                axialPower = -(robotAxialError / distance) * translationPower;
                lateralPower = -(robotLateralError / distance) * translationPower;
            }

            double yawPower = 0.0;
            if (!headingReached) {
                double turnLimit = Math.min(maxPower, 0.40);
                double turnPower = proportionalPower(Math.abs(headingError),
                        RobotConstants.HEADING_SLOWDOWN_DEG, RobotConstants.MIN_TURN_POWER, turnLimit);
                // Yoshi.drive() uses negative yaw for a positive OTOS turn.
                yawPower = -Math.signum(headingError) * turnPower;
            }

            robot.drive(axialPower, lateralPower, yawPower, 1.0);
            Log.d(TAG, String.format(
                    "t=%.2fs pose X=%.2f Y=%.2f H=%.2f | err dist=%.2f heading=%.2f | power axial=%.2f lateral=%.2f yaw=%.2f",
                    waypointTimer.seconds(), pose.x, pose.y, pose.h, distance, headingError,
                    axialPower, lateralPower, yawPower));
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

        Log.d(TAG, String.format("driveToPosition TIMEOUT t=%.2fs", waypointTimer.seconds()));
        robot.stopDrive();
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
