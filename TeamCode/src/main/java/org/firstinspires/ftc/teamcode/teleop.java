
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOP", group="OpMode")
public class teleop extends OpMode {

    // Declare OpMode members for each of the 4 motors.
    // Servo and motor declarations
    public Servo CLAW;
    public Servo rightArm;
    public Servo leftArm;
    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
//    public Drivetrain drivetrain;
    // Track button states
    public boolean currentAButton;
    public boolean previousAButton = false;
    public boolean currentXButton;
    public boolean previousXButton = false;
    public boolean currentYButton;
    public boolean previousYButton = false;

    // Servo positions (adjusted values)

    public float rightArmPos = 0.00f;
    public float CLAWOpen = 0.400f;
    public float CLAWClose = 0.00f;
    public float armRightPos = 0.00f;

    public boolean isclawopen=false;


    private double derivativeFilterAlpha = 0.1; // Filter coefficient (0 < alpha < 1)

    private double lastTime;

    //@Override
    public void runOpMode() throws InterruptedException {

    }

    @Override
    public void init() {
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

        CLAW.setPosition(CLAWClose);
    }

    //@Override
    public void loop() {
        // update button states
        currentAButton = gamepad1.a;
        boolean aButtonJustPressed = currentAButton && !previousAButton;

        currentXButton = gamepad1.x;
        boolean xButtonJustPressed = currentXButton && !previousXButton;

        currentYButton = gamepad1.y;
        boolean yButtonJustPressed = currentYButton && !previousYButton;

        // drive
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX = -gamepad1.left_stick_x;
        double rightStickX = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(leftStickY) + Math.abs(leftStickX) + Math.abs(rightStickX), 1);
        double leftFrontPower = (leftStickY + leftStickX + rightStickX) / denominator;
        double leftBackPower = (leftStickY - leftStickX + rightStickX) / denominator;
        double rightFrontPower = (leftStickY - leftStickX - rightStickX) / denominator;
        double rightBackPower = (leftStickY + leftStickX - rightStickX) / denominator;

        frontLeft.setPower(leftFrontPower);
        backLeft.setPower(leftBackPower);
        frontRight.setPower(rightFrontPower);
        backRight.setPower(rightBackPower);

        if (xButtonJustPressed){
            if(isclawopen){
                CLAW.setPosition(CLAWClose);
            }
            else{
                CLAW.setPosition(CLAWOpen);
            }
        }

        if(yButtonJustPressed){
            rightArm.setPosition(0.3);
            leftArm.setPosition(0.3);
        }
        if(aButtonJustPressed){
            rightArm.setPosition(0);
            leftArm.setPosition(0);
        }

        //servo 0
        if (gamepad1.a) {
        CLAW.setPosition(0.00);
        }
    }

    // PIDController class
    public class PIDController {
        private double kP, kI, kD;
        private double setpoint;
        private double integral;
        private double previousError;
        private double outputMin, outputMax;

        private double derivativeFilterAlpha;
        private double filteredDerivative;

        public PIDController(double kP, double kI, double kD, double outputMin, double outputMax, double derivativeFilterAlpha) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD; // Set to 0 if using filtered derivative
            this.outputMin = outputMin;
            this.outputMax = outputMax;
            this.derivativeFilterAlpha = derivativeFilterAlpha;
            integral = 0;
            previousError = 0;
            filteredDerivative = 0;
        }

        public void setSetpoint(double setpoint) {
            this.setpoint = setpoint;
        }

        public double update(double current, double deltaTime) {
            double error = setpoint - current;
            integral += error * deltaTime;

// Calculate raw derivative
            double derivative = (error - previousError) / deltaTime;

// Apply low-pass filter to derivative
            filteredDerivative = derivativeFilterAlpha * derivative + (1 - derivativeFilterAlpha) * filteredDerivative;

            previousError = error;

// Compute PID output using filtered derivative
            double output = kP * error + kI * integral + kD * filteredDerivative;
            output = Math.max(outputMin, Math.min(outputMax, output)); // Clamping the output

            return output;
        }
    }

}