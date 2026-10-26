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
        sleep(3000); // Todo: what does VelocityGraph tell us about this?
        robot.setIntakePower(RobotConstants.DEFAULT_INTAKE_SPEED);

        for (int i = 0; i < 4; i++) { // Todo: with the new pValue, what should we do here?
            robot.setTriggerPower(RobotConstants.TRIGGER_POWER);
            sleep(2000);
            robot.setTriggerPower(0.0);
            sleep(500);
        }

        // Todo: can you pick up the balls from the corner?
        robot.setShooterVelocity(0.0);
        robot.setIntakePower(0.0);

        robot.driveToPosition(this, 660.4, -431.8, 0.0, 0.6);
        robot.driveToPosition(this, 900, -1600, 0.0, 0.6);
    }
}