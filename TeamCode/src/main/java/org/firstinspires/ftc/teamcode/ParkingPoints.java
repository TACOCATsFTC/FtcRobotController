package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Parking Points", group = "Robot")
public class ParkingPoints extends LinearOpMode {
    private final Yoshi robot = new Yoshi();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to do parking auto");
        telemetry.update();

        waitForStart();

        robot.driveToPosition(this, 200.0, 200.0, 0.0, 0.6);
        SparkFunOTOS.Pose2D finalPose = robot.otos.getPosition();
        telemetry.addData("Path", "Complete");
        telemetry.addData("Final pose", "X %.1f in  Y %.1f in  H %.1f deg",
                finalPose.x, finalPose.y, finalPose.h);
        telemetry.update();
        sleep(1000);
    }
}
