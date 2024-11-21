package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Auton.AutonDirection;

public class Robot {
    private static final double ARM_ANGLE_ADJUSTMENT_FACTOR = 3;

    private static final double WRIST_ANGLE_ADJUSTMENT_FACTOR = 0.01;
    private static final float SLIDE_ADJUSTMENT_FACTOR = 3;
    private final Telemetry telemetry;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor parallelSensor;
    private final DcMotor perpendicularSensor;
    private final DcMotor arm;

    private final DcMotor slide;
    private final Servo claw;
    private final Servo wrist;

    private double armPosition;

    private double armAngleAdjustment;

    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
   To find this, we first need to consider the total gear reduction powering our arm.
   First, we have an external 20t:100t (5:1) reduction created by two spur gears.
   But we also have an internal gear reduction in our motor.
   The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
   reduction of ~50.9:1. (more precisely it is 250047/4913:1)
   We can multiply these two ratios together to get our final reduction of ~254.47:1.
   The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
   counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 3 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_HIGH   = 85 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 125 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 2  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double CLAW_OPEN    = 0.0;
    final double CLAW_CLOSED        =  0.9;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_OUT  = 0.5;
    final double WRIST_FOLDED_IN   = 0.1;

    public static double WHEEL_DIAMETER = 1.88976;
    public static final double TICKS_PER_REV = 2000;
    public double encTicksPerInches = TICKS_PER_REV/(WHEEL_DIAMETER*Math.PI);
    public double encInchesPerTicks = (WHEEL_DIAMETER*Math.PI)/TICKS_PER_REV;

    final double SLIDE_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    final double SLIDE_COLLAPSED = 0 * SLIDE_TICKS_PER_MM;
    final double SLIDE_SCORING_IN_HIGH_BASKET = 600 * SLIDE_TICKS_PER_MM;
    final double SLIDE_COLLECT = 275 * SLIDE_TICKS_PER_MM;

    final double SLIDE_SAFE_TO_STORE = 50 * SLIDE_TICKS_PER_MM;
    double slidePosition = SLIDE_COLLAPSED;
    private static final int ARM_SPEED_DEFAUlT = 1000;

    private static final int ARM_SPEED_HANG = 2100;

    public Robot(OpMode opMode) {
        this.telemetry = opMode.telemetry;
        HardwareMap hardwareMap = opMode.hardwareMap;
        // Initialize the drive system variables.
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        this.backLeft = hardwareMap.get(DcMotor.class, "RLM");
        this.backRight = hardwareMap.get(DcMotor.class, "RRM");
        this.frontLeft = hardwareMap.get(DcMotor.class, "FLM");
        this.frontRight = hardwareMap.get(DcMotor.class, "FRM");
        this.arm = hardwareMap.get(DcMotor.class, "ARM");
        this.slide = hardwareMap.get(DcMotor.class, "SLIDE");
        this.parallelSensor = frontRight;
        this.perpendicularSensor = frontLeft;

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // because we have four motors we need set all 4 motor directions
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection((DcMotorSimple.Direction.REVERSE));
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);


        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) arm).setCurrentAlert(5, CurrentUnit.AMPS);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /* Define and initialize servos.*/
        claw = hardwareMap.get(Servo.class, "CLAW");
        wrist  = hardwareMap.get(Servo.class, "WRIST");

        /* Make sure that the intake is off, and the wrist is folded in. */
        claw.setPosition(CLAW_CLOSED);
        wrist.setPosition(WRIST_FOLDED_IN);

        this.storeRobot();
        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");

    }

    public void teleOpDrive(double drive, double turn, double strafe, double powerFactor) {
        strafe = strafe * 1.1; // Counteract imperfect strafing
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = powerFactor * (drive + strafe + turn) / denominator;
        double backLeftPower = powerFactor * (drive - strafe + turn) / denominator;
        double frontRightPower = powerFactor * (drive - strafe - turn) / denominator;
        double backRightPower = powerFactor * (drive + strafe - turn) / denominator;

        // Send calculated power to wheels
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
        // Show the wheel power.
        telemetry.addData("Drive Parameters", "drive: %s, turn: %s, power: %s", drive, turn, powerFactor);

    }

    public void autoDrive(int inches, double speed, AutonDirection autonDirection) {

        double ticksToTravel = inches * encTicksPerInches;
        DcMotor sensorToUse;
        // for reverse we need to reverse the speed
        switch (autonDirection) {
            case left:
                frontLeft.setPower(-speed);
                backLeft.setPower(speed);
                frontRight.setPower(speed);
                backRight.setPower(-speed);
                sensorToUse = perpendicularSensor;
                break;
            case right:
                frontLeft.setPower(speed);
                backLeft.setPower(-speed);
                frontRight.setPower(-speed);
                backRight.setPower(speed);
                sensorToUse = perpendicularSensor;
                break;
            case forward:
                frontRight.setPower(speed);
                backLeft.setPower(speed);
                frontLeft.setPower(speed);
                backRight.setPower(speed);
                sensorToUse = parallelSensor;
                break;
            case reverse:
                frontRight.setPower(-speed);
                backLeft.setPower(-speed);
                frontLeft.setPower(-speed);
                backRight.setPower(-speed);
                sensorToUse = parallelSensor;
                break;
            default:
                throw new IllegalArgumentException("autonDirection is not defined: " + autonDirection);
        }
        // Reset the encoder on the proper deadwheel
        sensorToUse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sensorToUse.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime runtime = new ElapsedTime();

        // our odometry pod is reversed in orientation so going forward gives a negative value, while
        // going backward gives a positive value - using the absolute value here allows us to ignore this.
        while (Math.abs(sensorToUse.getCurrentPosition()) <= ticksToTravel) {
            telemetry.addData("Distance", "traveled: %4.1f Inches", sensorToUse.getCurrentPosition() / encInchesPerTicks);
            telemetry.addData("Path", "%4.1f S Elapsed", runtime.seconds());

        }
    }

    public void pause(double pauseTimeSeconds) {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < pauseTimeSeconds) {
            telemetry.addData("Pausing: %4.1f S Elapsed", runtime.seconds());

        }
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void openClaw() {
        claw.setPosition(CLAW_OPEN);
    }

    public void closeClaw() {
        claw.setPosition(CLAW_CLOSED);
    }

    public void deposit() {
        claw.setPosition(CLAW_OPEN);
    }

    public void collectSample() {
        armPosition = ARM_COLLECT;
        slidePosition = SLIDE_COLLECT;
        wrist.setPosition(WRIST_FOLDED_OUT);
        claw.setPosition(CLAW_OPEN);
        moveSlideToPosition();
        moveArmToPosition();
    }

    public void raiseForHighBasket() {
        claw.setPosition(CLAW_CLOSED);
        wrist.setPosition(WRIST_FOLDED_OUT);
        armPosition = ARM_SCORE_SAMPLE_IN_HIGH;
        slidePosition = SLIDE_SCORING_IN_HIGH_BASKET;
        moveSlideToPosition();
        moveArmToPosition();
    }

    public void storeRobot() {
        this.stop();
        armPosition = ARM_COLLAPSED_INTO_ROBOT;
        slidePosition = SLIDE_COLLAPSED;
        claw.setPosition(CLAW_CLOSED);
        wrist.setPosition(WRIST_FOLDED_IN);
        moveSlideToPosition();
        while (slide.getCurrentPosition() > SLIDE_SAFE_TO_STORE) {
            // just wait
            this.telemetry.addData("current slide position: ", slide.getCurrentPosition());
        }

        moveArmToPosition();
    }

    public void raiseArmForLowHang() {
        armPosition = ARM_ATTACH_HANGING_HOOK;
        claw.setPosition(CLAW_CLOSED);
        wrist.setPosition(WRIST_FOLDED_IN);
        slidePosition = SLIDE_COLLAPSED;
        moveSlideToPosition();
        moveArmToPosition();
    }

    public void liftRobot() {
        armPosition = ARM_WINCH_ROBOT;
        claw.setPosition(CLAW_CLOSED);
        wrist.setPosition(WRIST_FOLDED_IN);
        moveArmToPosition(ARM_SPEED_HANG);
    }

    public void adjustArmAngle(double adjustment) {
        telemetry.addData("Arm adjustment: %4.2f", adjustment);
        // 15 degree adjustment - this can be increased or decreased
        armAngleAdjustment = 15 * ARM_TICKS_PER_DEGREE * adjustment;
        moveArmToPosition();
    }
    public void adjustArmAngleContinuous(double adjustment) {
        telemetry.addData("Arm adjustment: %4.2f", adjustment * ARM_ANGLE_ADJUSTMENT_FACTOR * ARM_TICKS_PER_DEGREE);
        // subtract the adjustment to get the desired direction from a human perspective
        armPosition = armPosition - adjustment * ARM_ANGLE_ADJUSTMENT_FACTOR * ARM_TICKS_PER_DEGREE;
        moveArmToPosition();
    }

    public void adjustWristAngleContinuous(double adjustment) {
        // subtract the adjustment to get the desired direction from a human perspective
        wrist.setPosition(wrist.getPosition() - adjustment * WRIST_ANGLE_ADJUSTMENT_FACTOR);
    }

    public void adjustSlideContinuous(float adjustment) {
        telemetry.addData("Slide adjustment: %4.2f", -adjustment * SLIDE_ADJUSTMENT_FACTOR * SLIDE_TICKS_PER_MM);
        // subtract the adjustment to get the desired direction from a human perspective
        slidePosition = slidePosition - adjustment * SLIDE_ADJUSTMENT_FACTOR * SLIDE_TICKS_PER_MM;
        if (slidePosition > 2200) {
            slidePosition = 2200;
        }
        if (slidePosition < 65) {
            slidePosition = 65;
        }
        moveSlideToPosition();
    }
    private void moveArmToPosition() {
        moveArmToPosition(ARM_SPEED_DEFAUlT);

    }

    private void moveArmToPosition(int velocity) {
        arm.setTargetPosition((int) (armPosition + armAngleAdjustment));

        ((DcMotorEx) arm).setVelocity(velocity);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /* Check to see if our arm is over the current limit, and report via telemetry. */
        if (((DcMotorEx) arm).isOverCurrent()){
            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }
        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("armTarget: ", arm.getTargetPosition());
        telemetry.addData("arm Encoder: ", arm.getCurrentPosition());

    }
    private void moveSlideToPosition() {
        slide.setTargetPosition((int) slidePosition);
        ((DcMotorEx) slide).setVelocity(2100);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("slide Target: ", slide.getTargetPosition());
        telemetry.addData("slide Encoder: ", slide.getCurrentPosition());

    }

}