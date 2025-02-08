package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private double targetHeading = 0;         // Desired heading in degrees
    private final double kP = 0.001;             // Proportional constant (tune as needed)
    private final double kI = 0.01;             // Integral constant (tune as needed)
    private final double kD = 0.01;             // Derivative constant (tune as needed)
    private final double TURN_DEADBAND = 0.05;  // Deadband threshold for turning input

    // Store the previous error for derivative calculation
    private double previousError = 0;
    // A simple accumulator for the integral term. In a full implementation
    // you'd typically also implement a timer/dt for proper integration.
    private double errorSum = 0;
    private ElapsedTime runtime = new ElapsedTime();

    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
        telemetry = opMode.telemetry;
    }

    public void init(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft   = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRight  = hardwareMap.get(DcMotorEx.class, "rightRear");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft  = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Claw = hardwareMap.get(Servo.class, "intakeServo");

        leftArmServo  = hardwareMap.get(Servo.class, "leftArmServo");
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

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw();
    }

    public void resetHeading() {
        targetHeading = getHeading();
        // Reset the PID errors when resetting the heading.
        previousError = 0;
        errorSum = 0;
    }

    public void drive_power(double x, double y, double turn) {
        // Mechanum Drivetrain Calculation
        // https://youtu.be/gnSW2QpkGXQ?si=t1mTyWRYmS33ekDV
        double theta, power, sin, cos, max;
        double leftFront, rightFront, leftBack, rightBack;

        theta = Math.atan2(y, x);
        power = Math.hypot(x, y);
        sin = Math.sin(theta - Math.PI / 4);
        cos = Math.cos(theta - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        leftFront  = power * cos / max + turn;
        rightFront = power * sin / max - turn;
        leftBack   = power * sin / max + turn;
        rightBack  = power * cos / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            leftFront  /= (power + Math.abs(turn));
            rightFront /= (power + Math.abs(turn));
            leftBack   /= (power + Math.abs(turn));
            rightBack  /= (power + Math.abs(turn));
        }

        frontLeft.setPower(leftFront);
        backLeft.setPower(leftBack);
        frontRight.setPower(rightFront);
        backRight.setPower(rightBack);
    }

    /**
     * PID controller that computes a control output for a given error.
     *
     * @param error the current error
     * @param kP    proportional constant
     * @param kI    integral constant
     * @param kD    derivative constant
     * @return the computed PID output for turning
     */
    private double pidControl(double error, double kP, double kI, double kD) {
        // Accumulate the error for the integral term.
        errorSum += error;

        // Proportional term
        double pTerm = error * kP;

        // Integral term
        double iTerm = errorSum * kI;

        // Derivative term
        double dTerm = (error - previousError) * kD;

        // Update previous error for the next cycle.
        previousError = error;

        return pTerm + iTerm + dTerm;
    }

    public void drive_gyro(double x, double y, double turnInput) {
        double turn;
        if (Math.abs(turnInput) < TURN_DEADBAND) {
            double currentHeading = getHeading();
            double error = targetHeading - currentHeading;
            // Normalize error to range [-180, 180]
            while (error > 180) {
                error -= 360;
            }
            while (error <= -180) {
                error += 360;
            }
            if(Math.abs(error)<5){
                error=0;
            }

            // Use full PID to compute the turn value from the error.
            turn = (pidControl(error, kP, kI, kD)*-1)/100;
        } else {
            // If a turning command is provided, use that input and update target heading.
            turn = turnInput;
            targetHeading = getHeading();
            // Reset PID accumulators to ensure smooth transition when resuming PID control.
            previousError = 0;
            errorSum = 0;
        }
        telemetry.addData("Turn", turn);
//        drive_power(x, y, turn);
    }

    public void drive(double x, double y, double turn, boolean gyro) {
        // Adding simple deadband for turn input.
        if (Math.abs(turn) < 0.1) {
            turn = 0;
        }
        if (gyro) {
            drive_gyro(x, y, turn);
        } else {
            drive_power(x, y, turn);
        }
    }
    public void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void timedDrive(double x, double y, double turn, double time) {
        runtime.reset();
        while (runtime.milliseconds() < time && myOpMode.opModeIsActive()) {
            drive_power(x, y, turn);
        }
        stopDrive();
    }

}
