package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * RoadRunner Drive Hardware Class - no teleop drive methods are in this class
 *
 */



@Disabled
public class vvHardwareRR {


    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)

    public DcMotorEx leftArm;
    public DcMotorEx rightArm;
    public DcMotorEx pickUp;
    public DcMotorEx lift;
    public CRServo rightWheel;
    public CRServo leftWheel;
    public Servo rightClaw;

    public Servo drone;

    public IMU imu;

    public ColorSensor colorSensor;
    public DistanceSensor distFront;
    public DistanceSensor distRear;
    private ElapsedTime runtime = new ElapsedTime();

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    public static final double droneSet = 0.25;

    // All variables below are used for auton methods
    public static final int autonPickupIdle = -30; // the idle position for the pickup motor 109
    public static final int autonPickupHigh = -5; // the placing position for the pickup motor in the high position 148
    public static final int autonPickupLow = -27; // the placing position for the pickup motor in the low/forward position 5
    public static final int autonPickupStack = -42; // pickup location for getting the top pixel on the stack
    public static final int autonPickupDoor = -45; // pickup location to move the stage door up

    public static final double pickUpPwr = 0.7;

    public static final double armEPower = 0.8;
    final double pickupTime = 1; // the amount of time the pickup takes to activate in seconds
    final double armTime = 3; // the amount of time the arm takes to raise in seconds

    public static final int armIdle = 0; // -84

    public static final int armLow = 110; // the low encoder position for the arm -23

    public static final int armHigh = 395; // the high-overhead encoder position for the arm 329
    public static final int armStart = 25;
    public static final int armStack = 29; // arm position to pick up to top pixel on pixel stack
    public static final int armDoor = 125 ; // arm position to move the stage door up


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public vvHardwareRR(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftArm = myOpMode.hardwareMap.get(DcMotorEx.class, "armL");
        rightArm = myOpMode.hardwareMap.get(DcMotorEx.class, "armR");
        pickUp = myOpMode.hardwareMap.get(DcMotorEx.class, "pickUp");
        lift = myOpMode.hardwareMap.get(DcMotorEx.class, "lift");

        // Define Servos
        rightWheel = myOpMode.hardwareMap.crservo.get("RSW");
        leftWheel = myOpMode.hardwareMap.crservo.get("LSW");
        drone = myOpMode.hardwareMap.get(Servo.class,"drone");

        drone.scaleRange(0,1);
        drone.setDirection(Servo.Direction.REVERSE);
        drone.setPosition(droneSet);

        //define and initialize sensors
        //colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "CLR");

        //distFront = myOpMode.hardwareMap.get(DistanceSensor.class, "FDS");
        //distRear = myOpMode.hardwareMap.get(DistanceSensor.class, "RDS");

        //Set the motor directions
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);

        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pickUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pickUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pickUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.addData("Drone Servo", drone.getPosition());
        myOpMode.telemetry.update();
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param armPower driving power (-1.0 to 1.0)
     */
    public void moveArm(double armPower) {
        leftArm.setPower(armPower);
        rightArm.setPower(armPower);
    }
    /**
     * Pass the requested arm position and power to the arm drive motors
     *
     * @param armEPower driving power (-1.0 to 1.0)
     * @param armPosition full lift range is
     */
    public void armPos(int armPosition, double armEPower) {
        leftArm.setTargetPosition(armPosition);
        rightArm.setTargetPosition(armPosition);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(armEPower);
        rightArm.setPower(armEPower);
    }
    public void movePickUp(int pickUpLoc, double pickUpPwr) {
        pickUp.setTargetPosition(pickUpLoc);
        pickUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pickUp.setPower(pickUpPwr);
    }
    public void pwrPickUp(double pickUpPwr) {
        pickUp.setPower(pickUpPwr);
    }
    /**
     * Set the pickup servo powers
     *
     * @param LWPower
     * @param RWPower
     */
    public void setPickupPower(double LWPower, double RWPower) {
        leftWheel.setPower(LWPower);
        rightWheel.setPower(RWPower);
    }

    public void setDronePosition(double droneSet) {
        drone.setPosition(droneSet);
    }

    public void moveLiftEnc(int liftLoc) {
        lift.setTargetPosition(liftLoc);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
    }
}


