package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="auto mateo", group="Robot")
public class Auto extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private final Yoshi robot         = new Yoshi();

    private double routeStartX = 0;
    private double routeStartY = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();
        runtime.reset();

        robot.updateOdometry();
        routeStartX = robot.getXMM();
        routeStartY = robot.getYMM();

        driveToPosition(200, 0, 0, 0.6);
        sleep(2500);
        // driveToPosition(0, 0, 0, 0.6);
        // sleep(2500);
        // driveToPosition(0, 500, 0, 0.6);
        // sleep(2500);
        // driveToPosition(0, 0, 0, 0.6);

        telemetry.addData("Path", "Complete");
        telemetry.addData("Total X traveled (mm)", robot.getXMM() - routeStartX);
        telemetry.addData("Total Y traveled (mm)", robot.getYMM() - routeStartY);
        telemetry.update();
        sleep(1000);
    }

    // Drives to a specific field position and heading.
    // targetX is side-to-side in mm (positive = right), targetY is forward/backward in mm (positive = forward).
    // Both are measured from where the robot started the OpMode.
    // targetHeading is in degrees (0 = same direction as start, positive = clockwise).
    // speed controls how fast the robot drives (0.0 to 1.0).
    private void driveToPosition(double targetX, double targetY, double targetH, double speed) {
        final double POSITION_TOLERANCE_MM = 15.0;
        final double HEADING_TOLERANCE_DEG = 3.0;
        double timeout   = 10.0;
        double moveStart = runtime.seconds();

        while (opModeIsActive() && runtime.seconds() - moveStart < timeout) {

            // Get current location
            robot.updateOdometry();
            double currentX  = robot.getXMM();
            double currentY  = robot.getYMM();
            double currentH = robot.getHeadingDegrees();

            // Calculate distance to target
            double distanceToX = targetX - currentX;
            double distanceToY = targetY - currentY;
            double distanceToH = ((targetH - currentH + 180) % 360 + 360) % 360 - 180;

            // If we are at our target, stop
            if (Math.abs(distanceToX) < POSITION_TOLERANCE_MM && 
                Math.abs(distanceToY) < POSITION_TOLERANCE_MM && 
                Math.abs(distanceToH) < HEADING_TOLERANCE_DEG) {
                break;
            }

            // Calculate how fast the wheels should turn
            double hRad         = Math.toRadians(currentH);
            // Robot forward direction in field = (-sin h, cos h) for (X=strafe, Y=forward)
            // Robot right  direction in field = ( cos h, sin h)
            // robotAxial   = dot(robot_forward, (dX, dY)) = dY*cos(h) - dX*sin(h)
            // robotLateral = dot(robot_right,   (dX, dY)) = dX*cos(h) + dY*sin(h)
            double robotAxial   = distanceToY * Math.cos(hRad) - distanceToX * Math.sin(hRad);
            double robotLateral = distanceToX * Math.cos(hRad) + distanceToY * Math.sin(hRad);

            double distanceToTarget = Math.sqrt(distanceToX * distanceToX + distanceToY * distanceToY);
            double posScale     = Math.min(1.0, distanceToTarget / 150.0);
            double axialPower   = 0;
            double lateralPower = 0;
            if (distanceToTarget > POSITION_TOLERANCE_MM) {
                axialPower   = (robotAxial / distanceToTarget) * speed * posScale;
                lateralPower = (robotLateral / distanceToTarget) * speed * posScale;
            }
            double yawPower = -Math.signum(distanceToH) * speed * 0.5;

            // Log all the data
            Log.d("DriveToPos", String.format(
                    "X=%3.1f  Y=%3.1f  H=%3.1f | dX=%3.1f  dY=%3.1f  dH=%3.1f | ax=%5.2f  lat=%5.2f  yaw=%5.2f",
                    currentX, currentY, currentH, distanceToX, distanceToY, distanceToH, axialPower, lateralPower, yawPower));

            // Drive
            robot.drive(axialPower, lateralPower, yawPower, 1.0);
        }
        robot.stopDrive();
    }

        // Drives forward (positive mm) or backward (negative mm) the given distance.
    private void moveForwardAndBackward(double distanceMM, double speed) {
        robot.updateOdometry();
        double startX = robot.getXMM();
        double direction = distanceMM > 0 ? 1.0 : -1.0;
        double timeout = (Math.abs(distanceMM) / 200.0) + 3.0;
        double moveStart = runtime.seconds();
        robot.drive(direction * Math.abs(speed), 0, 0, 1.0);
        while (opModeIsActive() && runtime.seconds() - moveStart < timeout) {
            robot.updateOdometry();
            telemetry.addData("X traveled (mm)", robot.getXMM() - routeStartX);
            telemetry.addData("Y traveled (mm)", robot.getYMM() - routeStartY);
            telemetry.update();
            if (Math.abs(robot.getXMM() - startX) >= Math.abs(distanceMM)) {
                break;
            }
        }
        robot.stopDrive();
    }

    // Strafes left (positive mm) or right (negative mm) the given distance.
    private void moveLeftAndRight(double distanceMM, double speed) {
        robot.updateOdometry();
        double startY = robot.getYMM();
        double direction = distanceMM > 0 ? 1.0 : -1.0;
        double timeout = (Math.abs(distanceMM) / 200.0) + 3.0;
        double moveStart = runtime.seconds();
        robot.drive(0, direction * Math.abs(speed), 0, 1.0);
        while (opModeIsActive() && runtime.seconds() - moveStart < timeout) {
            robot.updateOdometry();
            telemetry.addData("X traveled (mm)", robot.getXMM() - routeStartX);
            telemetry.addData("Y traveled (mm)", robot.getYMM() - routeStartY);
            telemetry.update();
            if (Math.abs(robot.getYMM() - startY) >= Math.abs(distanceMM)) {
                break;
            }
        }
        robot.stopDrive();
    }

    // Turns clockwise (positive degrees) or counter-clockwise (negative degrees).
    private void turn(double degrees, double speed) {
        robot.updateOdometry();
        double startHeading = robot.getHeadingDegrees();
        double direction = degrees > 0 ? 1.0 : -1.0;
        double timeout = (Math.abs(degrees) / 90.0) * 3.0 + 2.0;
        double moveStart = runtime.seconds();
        robot.drive(0, 0, direction * Math.abs(speed), 1.0);
        while (opModeIsActive() && runtime.seconds() - moveStart < timeout) {
            robot.updateOdometry();
            double turned = robot.getHeadingDegrees() - startHeading;
            telemetry.addData("Target (deg)", degrees);
            telemetry.addData("Turned (deg)", turned);
            telemetry.update();
            if (Math.abs(turned) >= Math.abs(degrees)) {
                break;
            }
        }
        robot.stopDrive();
    }

    private void shoot(double shooterSpeed, double duration) {
        robot.setShooterPower(shooterSpeed);
        robot.fireTrigger();
        double shootStart = runtime.seconds();
        while (opModeIsActive() && runtime.seconds() - shootStart < duration) {
            sleep(10);
        }
        robot.stopTrigger();
        robot.setShooterPower(0);
    }

}
