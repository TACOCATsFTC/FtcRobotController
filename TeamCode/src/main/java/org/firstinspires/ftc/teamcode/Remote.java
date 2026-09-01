package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 1) Axial:    Driving forward and backward                Left-joystick Forward/Backward
 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 */

@TeleOp(name="Yoshi TeleOp", group="Linear OpMode")
public class Remote extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private final Yoshi robot = new Yoshi();

    private double maxWheelSpeed = RobotConstants.DEFAULT_MAX_WHEEL_SPEED;
    private double intakeSpeed  = RobotConstants.DEFAULT_INTAKE_SPEED;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized. Updated 6/25/26 at 11:16pm");
        telemetry.update();

        waitForStart();
        runtime.reset();

        robot.setIntakePower(RobotConstants.DEFAULT_INTAKE_SPEED);

        while (opModeIsActive()) {
            handleIntakeControls();
            handleWheelSpeedControls();
            handleJoysticks();
            sendTelemetry();
        }
    }

    private void handleIntakeControls() {
        if (gamepad1.y) {
            intakeSpeed = Math.min(1.0, intakeSpeed + RobotConstants.INTAKE_SPEED_STEP);
            robot.setIntakePower(intakeSpeed);
        }
        if (gamepad1.a) {
            intakeSpeed = Math.max(-1.0, intakeSpeed - RobotConstants.INTAKE_SPEED_STEP);
            robot.setIntakePower(intakeSpeed);
        }
    }

    private void handleWheelSpeedControls() {
        if (gamepad1.dpad_down) {
            maxWheelSpeed = RobotConstants.SLOW_WHEEL_SPEED;
        }
        else if (gamepad1.dpad_left) {
            maxWheelSpeed = RobotConstants.MEDIUM_WHEEL_SPEED;
        }
        else if (gamepad1.dpad_right) {
            maxWheelSpeed = RobotConstants.FAST_WHEEL_SPEED;
        }
        else if (gamepad1.dpad_up) {
            maxWheelSpeed = RobotConstants.TURBO_WHEEL_SPEED;
        }
    }

    private void handleJoysticks() {
        double axial   = gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
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

        telemetry.addData("Left stick X/Y", "%4.2f, %4.2f",
        gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Right stick X/Y", "%4.2f, %4.2f",
        gamepad1.right_stick_x, gamepad1.right_stick_y);

        telemetry.addData("Intake speed", "%4.2f", robot.intake.getPower());
        telemetry.update();
    }
}
