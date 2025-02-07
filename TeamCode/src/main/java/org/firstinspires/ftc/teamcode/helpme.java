package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="setto0", group="OpMode")
public class helpme extends OpMode {
    public Servo CLAW;
    public Servo rightArm;
    public Servo leftArm;
    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    @Override
    public void init(){
        Robot robot = new Robot(hardwareMap);

        frontLeft = robot.frontLeft;
        frontRight = robot.frontRight;
        backLeft = robot.backLeft;
        backRight = robot.backRight;

        liftRight = robot.liftRight;
        liftLeft = robot.liftLeft;

        CLAW = robot.CLAW;

        rightArm = robot.rightArmServo;
        leftArm = robot.leftArmServo;
    }
    public void loop(){
        if(gamepad1.a){
            rightArm.setPosition(0.5);
            leftArm.setPosition(0.5);
        }
        if(gamepad1.b){
            rightArm.setPosition(1);
            leftArm.setPosition(0);
        }
        if(gamepad1.x){
            rightArm.setPosition(0);
            leftArm.setPosition(1);
        }
    }
}
