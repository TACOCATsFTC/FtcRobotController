package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Yoshi {

    public DcMotor frontLeftDrive;
    public DcMotor backLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backRightDrive;
    public DcMotor intake;

    public void init(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_LEFT_DRIVE);
        backLeftDrive = hardwareMap.get(DcMotor.class, RobotConstants.BACK_LEFT_DRIVE);
        frontRightDrive = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_RIGHT_DRIVE);
        backRightDrive = hardwareMap.get(DcMotor.class, RobotConstants.BACK_RIGHT_DRIVE);
        intake = hardwareMap.get(DcMotor.class, RobotConstants.INTAKE);

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Mecanum drive. axial = forward/back, lateral = strafe, yaw = rotate.
     * Powers are normalized then scaled by maxWheelSpeed.
     */
    public void drive(double axial, double lateral, double yaw, double maxWheelSpeed) {
        double fl = axial + lateral - yaw;
        double fr = axial + lateral + yaw;
        double bl = axial - lateral - yaw;
        double br = axial - lateral + yaw;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeftDrive.setPower(fl * maxWheelSpeed);
        frontRightDrive.setPower(fr * maxWheelSpeed);
        backLeftDrive.setPower(bl * maxWheelSpeed);
        backRightDrive.setPower(br * maxWheelSpeed);
    }

    public void setWheelPowers(double fl, double fr, double bl, double br) {
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    public void stopDrive() {
        setWheelPowers(0, 0, 0, 0);
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }
}

