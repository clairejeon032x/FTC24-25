package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;



import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {
    private Telemetry telemetry;
    private LinearOpMode myOpMode = null;
    public static DcMotorEx frontLeft;
    public static DcMotorEx frontRight;
    public static DcMotorEx backLeft;
    public static DcMotorEx backRight;
    public static DcMotorEx liftRight;
    public static DcMotorEx liftLeft;
    public static Servo Claw;
    public static Servo rightArmServo;
    public static Servo leftArmServo;

    public static DcMotorEx climbMotor;

    public static IMU imu;

    private double targetHeading = 0;          // Desired heading in degrees
    private final double kP = 0.01;              // Proportional constant (tune as needed)
    private final double kI=0.01;
    private final double kD=0.01;
    private final double TURN_DEADBAND = 0.05;    // Deadband threshold for turning input

    private static double previousError=0;


    public RobotHardware(LinearOpMode opMode){
        myOpMode=opMode;
        telemetry=opMode.telemetry;
    }

    public void init(HardwareMap hardwareMap){
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRight = hardwareMap.get(DcMotorEx.class, "rightRear");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Claw = hardwareMap.get(Servo.class, "intakeServo");

        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");

        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");

        leftArmServo.setDirection(Servo.Direction.FORWARD);
        rightArmServo.setDirection(Servo.Direction.REVERSE);

        climbMotor = hardwareMap.get(DcMotorEx.class, "climbMotor");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
    }
    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw();
    }

    public void resetHeading(){
        targetHeading=getHeading();
    }
    public void drive_power(double x, double y, double turn) {
        //Mechanum Drivetrain
        ////https://youtu.be/gnSW2QpkGXQ?si=t1mTyWRYmS33ekDV
        double theta, power, sin, cos, max, leftFront, rightFront, leftBack, rightBack;
        theta = Math.atan2(y,x);
        power = Math.hypot(x,y);
        sin = Math.sin(theta-Math.PI/4);
        cos = Math.cos(theta-Math.PI/4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        leftFront = power * cos/max + turn;
        rightFront = power * sin/max - turn;
        leftBack = power * sin/max + turn;
        rightBack = power * cos/max - turn;

        if ((power + Math.abs(turn))>1){
            leftFront /= power + Math.abs(turn);
            rightFront /= power + Math.abs(turn);
            leftBack    /= power + Math.abs(turn);
            rightBack  /= power + Math.abs(turn);
        }
        frontLeft.setPower(leftFront);
        backLeft.setPower(leftBack);
        frontRight.setPower(rightFront);
        backRight.setPower(rightBack);
    }

    private double pidControl(double error, double kP, double kI, double kD) {
        // Proportional term
        double pTerm = error * kP;

        // Integral term (averaged over time)
        double iTerm = 0.1 * getPreviousError() + 0.9 * error; // adjust coefficients to your needs

        // Derivative term
        double dTerm = (error - getPreviousError()) * kD;

        return pTerm + iTerm + dTerm;
    }

    // Helper method to get previous error (for integral calculation)
    private double getPreviousError() {
        if (previousError == 0) { // first call, initialize with current error
            previousError = error;
        }
        return previousError;
    }

    public void drive_gyro(double x,double y,double turnInput){
        double turn;
        if (Math.abs(turnInput) < TURN_DEADBAND) {
            double currentHeading = getHeading();
            double error = targetHeading - currentHeading;
            // Normalize error to range [-180, 180]
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;
            turn = error * kP*-1;

        } else {
            // If turning is commanded, use the driver’s input and update the target heading.
            turn = turnInput;
            targetHeading = getHeading();
        }
        telemetry.addData("turn",turn);
        drive_power(x,y,turn);
    }

    public void drive(double x,double y,double turn,boolean gyro){
        if(Math.abs(turn)<0.1){
            turn=0;
        }
        if(gyro){
            drive_gyro(x,y,turn);
        }
        else{
            drive_power(x,y,turn);
        }
    }

}

