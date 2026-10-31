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
        telemetry.update();

        waitForStart();
     
        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pose = robot.otos.getPosition();
            telemetry.addData("X", "%.2f mm", DistanceUnit.MM.fromInches(pose.x));
            telemetry.addData("Y", "%.2f mm", DistanceUnit.MM.fromInches(pose.y));
            telemetry.addData("Heading", "%.2f deg", pose.h);
            telemetry.update();
        }
    }
}
