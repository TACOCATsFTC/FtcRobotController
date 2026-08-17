package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 */

@TeleOp(name="Yoshi TeleOp", group="Linear OpMode")
public class RemoteDecode extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private final YoshiDecode robot = new YoshiDecode();

    private double maxWheelSpeed = RobotConstantsDecode.DEFAULT_MAX_WHEEL_SPEED;
    private double shooterSpeed  = RobotConstantsDecode.DEFAULT_SHOOTER_SPEED;

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
            shooterSpeed = Math.min(1.0, shooterSpeed + RobotConstantsDecode.SHOOTER_SPEED_STEP);
            robot.setShooterPower(shooterSpeed);
            robot.setLedPosition(RobotConstantsDecode.LED_POSITION_A);
        }
        if (gamepad1.a) {
            shooterSpeed = Math.max(0.0, shooterSpeed - RobotConstantsDecode.SHOOTER_SPEED_STEP);
            robot.setShooterPower(shooterSpeed);
            robot.setLedPosition(RobotConstantsDecode.LED_POSITION_B);
        }
    }

    private void handleSpeedControls() {
        if (gamepad1.dpad_down) {
            maxWheelSpeed = RobotConstantsDecode.SLOW_WHEEL_SPEED;
        }
        if (gamepad1.dpad_left) {
            maxWheelSpeed = RobotConstantsDecode.MEDIUM_WHEEL_SPEED;
        }
        if (gamepad1.dpad_right) {
            maxWheelSpeed = RobotConstantsDecode.FAST_WHEEL_SPEED;
        }
        if (gamepad1.dpad_up) {
            maxWheelSpeed = RobotConstantsDecode.TURBO_WHEEL_SPEED;
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

        if (Math.abs(axial)   < RobotConstantsDecode.JOYSTICK_DEADBAND) {
            axial = 0;
        }
        if (Math.abs(lateral) < RobotConstantsDecode.JOYSTICK_DEADBAND) {
            lateral = 0;
        }
        if (Math.abs(yaw)     < RobotConstantsDecode.JOYSTICK_DEADBAND) {
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
