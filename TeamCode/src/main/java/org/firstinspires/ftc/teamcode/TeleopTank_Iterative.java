package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop Tank", group="Robot")
//@Disabled
public class RobotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    public DcMotor  leftfrontDrive   = null;
    public DcMotor  rightfrontDrive  = null;
    public DcMotor  leftbackDrive = null;
    public DcMotor  rightbackDrive    = null;
    public DcMotor  leftArm     = null;
    public DcMotor  rightArm     = null;
    public Servo    CLAW    = null;
    public Servo    armUp   = null;

    double clawOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.50 ;   // Run arm motor up at 50% power
    public static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        leftfrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftbackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightbackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        leftArm    = hardwareMap.get(DcMotor.class, "leftArm");
        rightArm = hardwareMap.get(DcMotor.class, "rightArm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.FORWARD);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        CLAW  = hardwareMap.get(Servo.class, "claw");
        armUp = hardwareMap.get(Servo.class, "armUp");
        CLAW.setPosition(MID_SERVO);
        armUp.setPosition(MID_SERVO);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double up;
        double down;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        up = gamepad1.left_trigger;
        down = gamepad1.right_trigger;

        leftfrontDrive.setPower(left);
        rightfrontDrive.setPower(right);
        leftbackDrive.setPower(left);
        rightbackDrive.setPower(right);

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        CLAW.setPosition(MID_SERVO + clawOffset);
        //  rightClaw.setPosition(MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y)
            leftArm.setPower(ARM_UP_POWER);
        else if (gamepad1.a)
            leftArm.setPower(ARM_DOWN_POWER);
        else
            leftArm.setPower(0.0);

        if (up>0.5){
            leftArm.setPower(0.5);
            rightArm.setPower(0.5);
        }
        else if (down>0.5){
            leftArm.setPower(-0.5);
            rightArm.setPower(-0.5);
        }
        else{
            leftArm.setPower(-0);
            rightArm.setPower(-0);
