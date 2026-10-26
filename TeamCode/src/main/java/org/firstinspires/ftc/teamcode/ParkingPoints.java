package org.firstinspires.ftc.teamcode;
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
        sleep(1000);
    }
}