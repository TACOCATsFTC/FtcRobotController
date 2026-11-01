package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ShooterAuto", group = "Robot")
public class ShooterAuto extends LinearOpMode {
    private final Yoshi robot = new Yoshi();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Aim at the hive before pressing Start");
        telemetry.update();

        waitForStart();

        robot.setShooterVelocity(RobotConstants.DEFAULT_SHOOTER_SPEED);
        sleep(1000);
        robot.setIntakePower(RobotConstants.DEFAULT_INTAKE_SPEED);

        Shooter();

        robot.driveToPosition(this, 900.5, -60, -90, 0.5); // pick up in the corner
        robot.driveToPosition(this, 480, -845, -90, 0.6);
        robot.driveToPosition(this, 250, -2000, -180, 0.6); // near the base
        robot.driveToPosition(this, -500, -2335, -180, 0.6); // near the flower
        robot.driveToPosition(this, -515, -2515, -180, 0.6); // shooting position

        Shooter();

        robot.setShooterVelocity(0.0);
        robot.setIntakePower(0.0);

        robot.driveToPosition(this, -500, -2335, -180, 0.6);
        robot.driveToPosition(this, 900, -1600, 0.0, 0.6);
    }

    void Shooter() {
        for (int i = 0; i < 2; i++) {
            robot.setTriggerPower(RobotConstants.TRIGGER_POWER);
            sleep(2000);
            robot.setTriggerPower(0.0);
            sleep(500);
        }
    }   
}