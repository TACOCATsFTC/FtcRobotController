package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Owns every piece of hardware on the robot and exposes high-level actions
 * (drive, shooter, trigger, LED). OpModes should not call hardwareMap directly.
 */
public class Yoshi {

    public DcMotor frontLeftDrive;
    public DcMotor backLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backRightDrive;
    public DcMotor shooter;
    public Servo   frontLed;
    public CRServo trigger1;
    public CRServo trigger2;
    public GoBildaPinpointDriver odometry;

    /** Call once at the top of every OpMode's runOpMode(). */
    public void init(HardwareMap hardwareMap) {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_LEFT_DRIVE);
        backLeftDrive   = hardwareMap.get(DcMotor.class, RobotConstants.BACK_LEFT_DRIVE);
        frontRightDrive = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_RIGHT_DRIVE);
        backRightDrive  = hardwareMap.get(DcMotor.class, RobotConstants.BACK_RIGHT_DRIVE);
        shooter         = hardwareMap.get(DcMotor.class, RobotConstants.SHOOTER);

        frontLed = hardwareMap.get(Servo.class,   RobotConstants.FRONT_LED);
        trigger1 = hardwareMap.get(CRServo.class, RobotConstants.TRIGGER_1);
        trigger2 = hardwareMap.get(CRServo.class, RobotConstants.TRIGGER_2);

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, RobotConstants.PINPOINT);
        odometry.setOffsets(RobotConstants.ODO_X_OFFSET_MM, RobotConstants.ODO_Y_OFFSET_MM, DistanceUnit.MM);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometry.resetPosAndIMU();
    }

    /**
     * Mecanum drive. axial = forward/back, lateral = strafe, yaw = rotate.
     * Powers are normalized then scaled by maxWheelSpeed.
     */
    // Formula matches the original Remote OpMode; revisit if strafing feels wrong.
    public void drive(double axial, double lateral, double yaw, double maxWheelSpeed) {
        double fl = axial + lateral - yaw;
        double fr = axial + lateral + yaw;
        double bl = axial - lateral - yaw;
        double br = axial - lateral + yaw;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                              Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        frontLeftDrive .setPower(fl * maxWheelSpeed);
        frontRightDrive.setPower(fr * maxWheelSpeed);
        backLeftDrive  .setPower(bl * maxWheelSpeed);
        backRightDrive .setPower(br * maxWheelSpeed);
    }

    /** Set each wheel power independently (used for autonomous primitives). */
    public void setWheelPowers(double fl, double fr, double bl, double br) {
        frontLeftDrive .setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive  .setPower(bl);
        backRightDrive .setPower(br);
    }

    public void stopDrive() {
        setWheelPowers(0, 0, 0, 0);
    }

    // Shooter
    public void setShooterPower(double power) {
        shooter.setPower(power);
    }

    // Trigger
    public void fireTrigger() {
        trigger1.setPower(1);
        trigger2.setPower(-1);
    }

    public void stopTrigger() {
        trigger1.setPower(0);
        trigger2.setPower(0);
    }

    // LED
    public void setLedPosition(double position) {
        frontLed.setPosition(position);
    }

    // Odometry
    public void updateOdometry() {
        odometry.update();
    }

    public Pose2D getPosition() {
        return odometry.getPosition();
    }

    public double getHeadingDegrees() {
        return odometry.getPosition().getHeading(AngleUnit.DEGREES);
    }

    public double getXMM() {
        return odometry.getPosition().getX(DistanceUnit.MM);
    }

    public double getYMM() {
        return odometry.getPosition().getY(DistanceUnit.MM);
    }
}
