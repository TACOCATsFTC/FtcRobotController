package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Yoshi {
    private static final String TAG = "Yoshi";
    private static final double MILLIMETERS_PER_INCH = 25.4;

    public DcMotor frontLeftDrive;
    public DcMotor backLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backRightDrive;
    public DcMotor intake;
    public DcMotorEx shooter;
    public PIDFCoefficients shooterPIDF;
    public CRServo trigger;
    public SparkFunOTOS otos;

    public void init(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_LEFT_DRIVE);
        backLeftDrive = hardwareMap.get(DcMotor.class, RobotConstants.BACK_LEFT_DRIVE);
        frontRightDrive = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_RIGHT_DRIVE);
        backRightDrive = hardwareMap.get(DcMotor.class, RobotConstants.BACK_RIGHT_DRIVE);

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, RobotConstants.INTAKE);
        trigger = hardwareMap.get(CRServo.class, RobotConstants.TRIGGER);

        shooter = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setShooterPValue(RobotConstants.DEFAULT_SHOOTER_P);

        otos = hardwareMap.get(SparkFunOTOS.class, RobotConstants.OTOS_NAME);
        configureOtos();
    }

    private void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(
                RobotConstants.OTOS_OFFSET_X_IN, RobotConstants.OTOS_OFFSET_Y_IN,
                RobotConstants.OTOS_OFFSET_HEADING_DEG));
        otos.setLinearScalar(RobotConstants.OTOS_LINEAR_SCALAR);
        otos.setAngularScalar(RobotConstants.OTOS_ANGULAR_SCALAR);
        otos.calibrateImu();
        otos.resetTracking();
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void setShooterVelocity(double velocity) {
        shooter.setVelocity(velocity);
    }

    public void setShooterPValue(double p) {
        shooterPIDF = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterPIDF.p = p;
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
    }

    public void setTriggerPower(double power) {
        trigger.setPower(power);
    }

    public void drive(double axial, double lateral, double yaw, double maxWheelSpeed) {
        double fl = axial + lateral - yaw;
        double fr = axial - lateral + yaw;
        double bl = axial - lateral - yaw;
        double br = axial + lateral + yaw;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeftDrive.setPower(fl * maxWheelSpeed); // todo: this should be a call to setWheelPowers()
        frontRightDrive.setPower(fr * maxWheelSpeed);
        backLeftDrive.setPower(bl * maxWheelSpeed);
        backRightDrive.setPower(br * maxWheelSpeed);
    }

    public void setWheelPowers(double fl, double fr, double bl, double br) { // todo: this should be private
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    public void stopDrive() { // todo: this should be private
        setWheelPowers(0, 0, 0, 0);
    }

    public void driveToPosition(LinearOpMode opMode, double targetXMillimeters,
                                double targetYMillimeters,
                                double targetHeading, double speed) {
        double targetX = targetXMillimeters / MILLIMETERS_PER_INCH;
        double targetY = targetYMillimeters / MILLIMETERS_PER_INCH;
        ElapsedTime moveTimer = new ElapsedTime();

        while (opMode.opModeIsActive()
                && moveTimer.seconds() < RobotConstants.WAYPOINT_TIMEOUT_SECONDS) {
            SparkFunOTOS.Pose2D pose = otos.getPosition();
            double distanceToX = targetX - pose.x;
            double distanceToY = targetY - pose.y;
            double distanceToHeading = normalizeDegrees(targetHeading - pose.h);
            double distanceToTarget = Math.hypot(distanceToX, distanceToY);

            boolean positionReached =
                    distanceToTarget <= RobotConstants.POSITION_TOLERANCE_IN;
            boolean headingReached =
                    Math.abs(distanceToHeading) <= RobotConstants.HEADING_TOLERANCE_DEG;
            if (positionReached && headingReached) {
                break;
            }

            double headingRadians = Math.toRadians(pose.h);
            double robotAxial = distanceToY * Math.cos(headingRadians)
                    - distanceToX * Math.sin(headingRadians);
            double robotLateral = distanceToX * Math.cos(headingRadians)
                    + distanceToY * Math.sin(headingRadians);

            double axialPower = 0.0;
            double lateralPower = 0.0;
            if (!positionReached) {
                double positionScale = Math.min(
                        1.0, distanceToTarget / RobotConstants.POSITION_SLOWDOWN_IN);
                double drivePower = Math.max(
                        RobotConstants.MIN_DRIVE_POWER, speed * positionScale);
                axialPower = (robotAxial / distanceToTarget) * drivePower;
                lateralPower = -(robotLateral / distanceToTarget) * drivePower;
            }

            double yawPower = 0.0;
            if (!headingReached) {
                double headingScale = Math.min(
                        1.0, Math.abs(distanceToHeading) / RobotConstants.HEADING_SLOWDOWN_DEG);
                double turnPower = Math.max(
                        RobotConstants.MIN_TURN_POWER, Math.min(speed * 0.5, speed * headingScale));
                yawPower = -Math.signum(distanceToHeading) * turnPower;
            }

            Log.d(TAG, String.format(
                    "X=%.1f Y=%.1f H=%.1f | dX=%.1f dY=%.1f dH=%.1f | ax=%.2f lat=%.2f yaw=%.2f",
                    pose.x, pose.y, pose.h, distanceToX, distanceToY, distanceToHeading,
                    axialPower, lateralPower, yawPower));

            drive(axialPower, lateralPower, yawPower, 1.0);
            opMode.telemetry.addData("Target", "X %.1f  Y %.1f  H %.1f",
                    targetX, targetY, targetHeading);
            opMode.telemetry.addData("Position", "X %.1f  Y %.1f  H %.1f",
                    pose.x, pose.y, pose.h);
            opMode.telemetry.update();
            opMode.idle();
        }

        stopDrive();
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
