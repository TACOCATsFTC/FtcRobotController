package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "OTOS Test Remote", group = "Test") // todo: disable before the competition
public class OtosTestRemote extends LinearOpMode {
    private final Yoshi robot = new Yoshi();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addLine("Ready");
        telemetry.addLine("Press A to reset the OTOS position");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        boolean resetButtonWasPressed = false;

        while (opModeIsActive()) {
            double axial = applyDeadband(-gamepad1.left_stick_y);
            double lateral = applyDeadband(-gamepad1.left_stick_x);
            double yaw = applyDeadband(gamepad1.right_stick_x);
            robot.drive(axial, lateral, yaw, RobotConstants.DEFAULT_MAX_WHEEL_SPEED);

            if (gamepad1.a && !resetButtonWasPressed) {
                robot.otos.resetTracking();
                robot.otos.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
            }
            resetButtonWasPressed = gamepad1.a;

            SparkFunOTOS.Pose2D pose = robot.otos.getPosition();
            telemetry.addData("X", "%.2f mm", DistanceUnit.MM.fromInches(pose.x));
            telemetry.addData("Y", "%.2f mm", DistanceUnit.MM.fromInches(pose.y));
            telemetry.addData("Heading", "%.2f deg", pose.h);
            telemetry.addData("Drive", "Axial %.2f  Lateral %.2f  Yaw %.2f",
                    axial, lateral, yaw);
            telemetry.addLine("Press A to reset the OTOS position");
            telemetry.update();
        }

        robot.stopDrive();
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < RobotConstants.JOYSTICK_DEADBAND ? 0.0 : value;
    }
}
