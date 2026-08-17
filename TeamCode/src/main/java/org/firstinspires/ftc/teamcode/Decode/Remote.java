package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 */

@TeleOp(name="Yoshi TeleOp", group="Linear OpMode")
public class Remote extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private final Yoshi robot = new Yoshi();

    private double maxWheelSpeed = RobotConstants.DEFAULT_MAX_WHEEL_SPEED;
    private double shooterSpeed  = RobotConstants.DEFAULT_SHOOTER_SPEED;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized. Updated 6/25/26 at 11:16pm");
        telemetry.update();

        waitForStart();
        runtime.reset();

        robot.setShooterPower(shooterSpeed);

        while (opModeIsActive()) {
            handleShooterControls();
            handleSpeedControls();
            handleTriggerControls();
            handleJoysticks();
            sendTelemetry();
        }
    }

    private void handleShooterControls() {
        if (gamepad1.y) {
            shooterSpeed = Math.min(1.0, shooterSpeed + RobotConstants.SHOOTER_SPEED_STEP);
            robot.setShooterPower(shooterSpeed);
            robot.setLedPosition(RobotConstants.LED_POSITION_A);
        }
        if (gamepad1.a) {
            shooterSpeed = Math.max(0.0, shooterSpeed - RobotConstants.SHOOTER_SPEED_STEP);
            robot.setShooterPower(shooterSpeed);
            robot.setLedPosition(RobotConstants.LED_POSITION_B);
        }
    }

    private void handleSpeedControls() {
        if (gamepad1.dpad_down) {
            maxWheelSpeed = RobotConstants.SLOW_WHEEL_SPEED;
        }
        if (gamepad1.dpad_left) {
            maxWheelSpeed = RobotConstants.MEDIUM_WHEEL_SPEED;
        }
        if (gamepad1.dpad_right) {
            maxWheelSpeed = RobotConstants.FAST_WHEEL_SPEED;
        }
        if (gamepad1.dpad_up) {
            maxWheelSpeed = RobotConstants.TURBO_WHEEL_SPEED;
        }
    }

    private void handleTriggerControls() {
        if (gamepad1.right_trigger > 0) {
            robot.fireTrigger();
        } else {
            robot.stopTrigger();
        }
    }

    private void handleJoysticks() {
        double axial   = -gamepad1.left_stick_y;  // pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw     = gamepad1.right_stick_x;

        if (Math.abs(axial)   < RobotConstants.JOYSTICK_DEADBAND) {
            axial = 0;
        }
        if (Math.abs(lateral) < RobotConstants.JOYSTICK_DEADBAND) {
            lateral = 0;
        }
        if (Math.abs(yaw)     < RobotConstants.JOYSTICK_DEADBAND) {
            yaw = 0;
        }

        robot.drive(axial, lateral, yaw, maxWheelSpeed);
    }

    private void sendTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Wheel max power", "%4.2f", maxWheelSpeed);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f",
                robot.frontLeftDrive.getPower(), robot.frontRightDrive.getPower());
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f",
                robot.backLeftDrive.getPower(), robot.backRightDrive.getPower());
        telemetry.addData("Front LED", "%4.2f", robot.frontLed.getPosition());
        telemetry.addData("Shooter speed", "%4.2f", robot.shooter.getPower());
        telemetry.update();
    }
}
