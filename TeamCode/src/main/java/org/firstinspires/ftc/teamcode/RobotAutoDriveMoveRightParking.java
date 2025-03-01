package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Robot: Auto Drive Move Right Parking", group="Robot")
public class RobotAutoDriveMoveRightParking extends LinearOpMode {

    public RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.timedDrive(0.8, 0, 0, 1000);
        robot.stopDrive();
    }
}
