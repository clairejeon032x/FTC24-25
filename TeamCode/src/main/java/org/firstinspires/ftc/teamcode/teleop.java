package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="teleop", group="Linear OpMode")

public class teleop extends LinearOpMode {
    public RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        double leftFront, rightFront, leftBack, rightBack, larm, rarm, clawPos, climb;
        boolean aButtonJustPressed, xButtonJustPressed, yButtonJustPressed;

        double strafe_k = 0.19;
        double turn_k = 0.25;
        double straight_k = 0.1;
        double x, y, turn, theta, power,sin,cos,max;

        boolean grab;
        grab = true;
        int grabInt = 0;
        robot.Claw.setPosition(0.58);

        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();
        waitForStart();
        robot.resetHeading();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            currentGamepad = gamepad1;

            aButtonJustPressed = currentGamepad.a && !previousGamepad.a;
            xButtonJustPressed = currentGamepad.x && !previousGamepad.x;
            yButtonJustPressed = currentGamepad.y && !previousGamepad.y;

            /*** Mechanum Drive ***/

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            robot.drive(x,y,turn,true);



            //claw close 0.41
            //claw open 0.58



            if (currentGamepad.a && !previousGamepad.a){
                grabInt += 1;
                grabInt = grabInt % 2;

            }

            if (gamepad1.a){
                robot.Claw.setPosition(0.41);
            } else {
                robot.Claw.setPosition(0.58);
            }

            if (gamepad1.left_bumper){

                larm = 0.01;
                rarm = 0.01;
            } else {
                larm = 0.32;
                rarm = 0.32;
            }


            if (gamepad1.right_trigger>0.01){
                robot.liftLeft.setPower(gamepad1.right_trigger);
                robot.liftRight.setPower(gamepad1.right_trigger);
            }
            else if (gamepad1.right_bumper){
                robot.liftLeft.setPower(-0.4);
                robot.liftRight.setPower(-0.4);

            }else {
                robot.liftLeft.setPower(0);
                robot.liftRight.setPower(0);
            }



            // larm = gamepad1.left_trigger;
            // rarm = gamepad1.left_trigger;
            robot.leftArmServo.setPosition(larm);
            robot.rightArmServo.setPosition(rarm);

            if(gamepad1.dpad_up){
                robot.climbMotor.setPower(0.5);
            }
            else if(gamepad1.dpad_down){
                robot.climbMotor.setPower(-0.5);
            }
            else{
                robot.climbMotor.setPower(0);
            }

            previousGamepad = currentGamepad;
            //open claw 0.58
            //close claw 0.41

            //minPower = 0.04



            telemetry.addData("armPosition", gamepad1.left_trigger);
            telemetry.addData("imu", robot.imu.getRobotYawPitchRollAngles().toString());
            telemetry.update();

        }
    }

}
