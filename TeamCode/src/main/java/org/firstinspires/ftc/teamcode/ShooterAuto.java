package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ShooterAuto", group = "Robot")
public class ShooterAuto extends LinearOpMode {
    private static final String TAG = "ShooterAuto";
    private final Yoshi robot = new Yoshi();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Status", "Aim at the hive before pressing Start");
        telemetry.update();

        waitForStart();

        robot.setShooterPower(RobotConstants.DEFAULT_SHOOTER_SPEED);
        sleep(2000);
        robot.setIntakePower(RobotConstants.DEFAULT_INTAKE_SPEED);

        for (int i = 0; i < 5; i++) {
            robot.setTriggerPower(RobotConstants.TRIGGER_POWER);
            sleep(2000);
            robot.setTriggerPower(0.0);
            sleep(500);
        }

        robot.setShooterPower(0.0);
        robot.setIntakePower(0.0);

        robot.driveToPosition(this, 660.4, -431.8, 0.0, 0.6);
        robot.driveToPosition(this, 800, -1500, 0.0, 0.6);

        return;
    }
}
