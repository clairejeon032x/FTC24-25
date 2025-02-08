package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Robot: Auto Drive Move Left Parking", group="Robot")
public class RobotAutoDriveMoveLeftParking extends LinearOpMode {
    public RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.timedDrive(-0.5, 0, 0, 1000);
        robot.stopDrive();
    }
}
