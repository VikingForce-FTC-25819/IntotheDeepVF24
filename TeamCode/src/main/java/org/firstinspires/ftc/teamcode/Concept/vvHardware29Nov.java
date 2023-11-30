/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Concept;

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
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes four motors (left_front, right_front, left_back, right_back)
 * Sensors three (2M distance x2, color)
 *
 * This one file/class can be used by ALL of your OpModes.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 */

@Disabled
public class vvHardware29Nov {


    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx rightRear;
    public DcMotorEx leftRear;
    public DcMotorEx leftArm;
    public DcMotorEx rightArm;
    public DcMotorEx pickUp;
    public DcMotorEx lift;
    public CRServo rightWheel;
    public CRServo leftWheel;
    public Servo rightClaw;

    public Servo drone;

    public IMU imu;
    public DcMotor parallelEncoder;
    public DcMotor perpendicularEncoder;
    public ColorSensor colorSensor;
    public DistanceSensor distFront;
    public DistanceSensor distRear;
    private ElapsedTime runtime = new ElapsedTime();

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double clawClose      =  1 ;
    public static final double clawOpen       =  0 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double droneSet = 0.25;

    // All variables below are used for auton methods
    final int autonPickupIdle = -30; // the idle position for the pickup motor 109
    final int autonPickupHigh = -5; // the placing position for the pickup motor in the high position 148
    final int autonPickupLow = -27; // the placing position for the pickup motor in the low/forward position 5



    // the amount of time the pickup takes to activate in seconds
    final double pickupTime = 1;
    // the amount of time the arm takes to raise in seconds
    final double armTime = 3;

    final public int armIdle = 0; // -84
    final public int armLow = 100; // the low encoder position for the arm -23
    final public int armHigh = 401; // the high-overhead encoder position for the arm 329
    final public int armStart = 25;
    final public int autonArmIdle = 5;

    static final double FORWARD_SPEED = 0.3;
    static final double TURN_SPEED = 0.5;
    public static double WHEEL_DIAMETER = 1.88976; // in
    public static final double TICKS_PER_REV = 2000;
    public double encTicksToInches = TICKS_PER_REV/(WHEEL_DIAMETER*Math.PI);
    static final double s1Side = 15;
    static final double s1Top = 20;
    static final double s2Turn = 45; //degrees
    static final double s3Reverse = 4;
    static final double s3Forward = 4;

    static final double s5 = 3.5;
    static final double s9Turn = 90; //Turn towards backdrop
    static final double s11 = 24; //Strafing distance to backdrop
    static final double s12 = 6; //Final distance to backdrop for placement
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public vvHardware29Nov(LinearOpMode opmode) {
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
        leftFront = myOpMode.hardwareMap.get(DcMotorEx.class, "FLM");
        rightFront = myOpMode.hardwareMap.get(DcMotorEx.class, "FRM");
        rightRear = myOpMode.hardwareMap.get(DcMotorEx.class, "RRM");
        leftRear = myOpMode.hardwareMap.get(DcMotorEx.class, "RLM");
        leftArm = myOpMode.hardwareMap.get(DcMotorEx.class, "armL");
        rightArm = myOpMode.hardwareMap.get(DcMotorEx.class, "armR");
        pickUp = myOpMode.hardwareMap.get(DcMotorEx.class, "pickUp");
        lift = myOpMode.hardwareMap.get(DcMotorEx.class, "lift");

        //Shadow the motors with encoder-odometry
        perpendicularEncoder = leftFront;
        parallelEncoder = rightFront; //Will need to use an opposite sign for right

        // Define Servos
        rightWheel = myOpMode.hardwareMap.crservo.get("RSW");
        //rightClaw = myOpMode.hardwareMap.get(Servo.class,"RSW");
        leftWheel = myOpMode.hardwareMap.crservo.get("LSW");
        drone = myOpMode.hardwareMap.get(Servo.class,"drone");

        //rightClaw.scaleRange(0.2,0.7);
        //rightClaw.setDirection(Servo.Direction.FORWARD);
        //rightClaw.setPosition(clawClose);

        drone.scaleRange(0,1);
        drone.setDirection(Servo.Direction.REVERSE);
        drone.setPosition(droneSet);

        //define and initialize sensors
        //colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "CLR");

        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw(); //reset the imu during initialization

        //distFront = myOpMode.hardwareMap.get(DistanceSensor.class, "FDS");
        //distRear = myOpMode.hardwareMap.get(DistanceSensor.class, "RDS");

        //Set the motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);

        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pickUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pickUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pickUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.addData("Drone Servo", drone.getPosition());
        myOpMode.telemetry.update();
    }
    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial-X motion), Strafe (Side-to-Side-Y motion) and Turn (Yaw-Z motion).
     * Then sends these power levels to the motors.
     * @param drivePower Global variable to set total drive power
     * @param driveY    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     *                  ..
     * @param strafe    Side to side driving power (-1.0 to 1.0) +ve is left
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double drivePower, double driveY, double strafe, double turn) {
        double strafeBias = 1; // given weight distribution need to bias the back wheel power
        double denominator = Math.max(Math.abs(driveY) + Math.abs(strafe*strafeBias) + Math.abs(turn), 1);
        double frontLeftPower = (driveY + strafe + turn) / denominator;
        double backLeftPower = (driveY - (strafe*strafeBias) + turn) / denominator;
        double frontRightPower = (driveY - strafe - turn) / denominator;
        double backRightPower = (driveY + (strafe*strafeBias) - turn) / denominator;

        leftFront.setPower(drivePower * frontLeftPower);
        leftRear.setPower(drivePower * backLeftPower);
        rightFront.setPower(drivePower * frontRightPower);
        rightRear.setPower(drivePower * backRightPower);
    }
    /**
     * Field centric method
     * robot motions: Drive (Axial-X motion), Strafe (Side-to-Side-Y motion) and Turn (Yaw-Z motion).
     * Then sends these power levels to the motors.
     * @param drivePower Global variable to set total drive power
     * @param driveYfc    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param strafeFC    Side to side driving power (-1.0 to 1.0) +ve is left
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobotFC(double drivePower, double driveYfc, double strafeFC, double turn) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double driveY = driveYfc * Math.cos(botHeading) - strafeFC * Math.sin(botHeading);
        double strafe = driveYfc * Math.sin(botHeading) + strafeFC * Math.cos(botHeading);

        double strafeBias = 1; // given weight distribution need to bias the back wheel power
        double denominator = Math.max(Math.abs(driveY) + Math.abs(strafe*strafeBias) + Math.abs(turn), 1);
        double frontLeftPower = (driveY + strafe + turn) / denominator;
        double backLeftPower = (driveY - (strafe*strafeBias) + turn) / denominator;
        double frontRightPower = (driveY - strafe - turn) / denominator;
        double backRightPower = (driveY + (strafe*strafeBias) - turn) / denominator;

        leftFront.setPower(drivePower * frontLeftPower);
        leftRear.setPower(drivePower * backLeftPower);
        rightFront.setPower(drivePower * frontRightPower);
        rightRear.setPower(drivePower * backRightPower);
    }
    /**
     * Sets the target for the chassis to achieve the desired position and heading
     * robot motions: Drive (Axial-X motion), Strafe (Side-to-Side-Y motion) and Turn (Yaw-Z motion).
     * Then sends these power levels to the motors.
     * @param autonPower Global variable to set total drive power
     * @param driveY    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param strafe    Side to side driving power (-1.0 to 1.0) +ve is left
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveAuton(double autonPower, double driveY, double strafe, double turn) {
        double denominator = Math.max(Math.abs(driveY) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (driveY + strafe + turn) / denominator;
        double backLeftPower = (driveY - strafe + turn) / denominator;
        double frontRightPower = (driveY - strafe - turn) / denominator;
        double backRightPower = (driveY + strafe - turn) / denominator;

        leftFront.setPower(autonPower * frontLeftPower);
        leftRear.setPower(autonPower * backLeftPower);
        rightFront.setPower(autonPower * frontRightPower);
        rightRear.setPower(autonPower * backRightPower);
    }
    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param armPower driving power (-1.0 to 1.0)
     */
    public void moveArm(double armPower) {
        leftArm.setPower(armPower);
        rightArm.setPower(armPower);
        /*if (leftArm.getCurrentPosition() >80 && leftArm.getCurrentPosition()<140) {
            leftArm.setPower(armPower * 0.5);
            rightArm.setPower(armPower * 0.5);
        }
        else {
            leftArm.setPower(armPower);
            rightArm.setPower(armPower);
        }*/
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

    /**
     * Set the claw servo to close
     *
     * @param clawClose
     */
    public void setRightClawPosition(double clawClose) {
        rightClaw.setPosition(clawClose);
    }
    /**
     * Set the drone servo
     *
     * @param droneSet
     */

    public void setDronePosition(double droneSet) {
        drone.setPosition(droneSet);
    }

    /**
     * Pass the requested lift power to the appropriate hardware drive motor
     *
     * @param liftPower driving power (-1.0 to 1.0)
     */
    public void moveLift(double liftPower) {
        lift.setPower(liftPower); }
    /**
     * Pass the requested lift position to the appropriate hardware drive motor
     * 1250 ticks will clear the bar
     * @param liftLoc driving power (-1.0 to 1.0)
     */
    public void moveLiftEnc(int liftLoc) {
        lift.setTargetPosition(liftLoc);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
    }
    public void autonStageTop() {
        // Wait for the game to start (driver presses PLAY)
        while (myOpMode.opModeIsActive()) {
            // Step -1 ; Mover arm u


            //Step 0; Move pickup up

           /* movePickUp(autonPickupHigh, 0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1.0)) {
                myOpMode.telemetry.addData("Path", "Leg 0: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
*/
            // Step 1:  Drive forward for 2 seconds or on ticks

            driveRobot(1, FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 1.7)||(parallelEncoder.getCurrentPosition()<(s1Top*encTicksToInches)))) {
                myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 2:  Spin right for 1.3 seconds

            driveRobot(0.3, 0, 0, 0); //No Turn
            final double s1Finish = parallelEncoder.getCurrentPosition();

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1)) {
                myOpMode.telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 3:  Drive Backward for 0.6 Second

            driveRobot(1, -FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 0.6)||(parallelEncoder.getCurrentPosition()>(s1Finish-(s3Reverse*encTicksToInches))))) {
                myOpMode.telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 4:  Stop
            driveRobot(0, 0, 0, 0);
            final double s3Finish = parallelEncoder.getCurrentPosition();

            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1.0)) {
                myOpMode.telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            // Step 5 unhook the pickup

            movePickUp(5,0.5);

            armPos(armStart,0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 2.0)) {
                myOpMode.telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            // Step 5.5: Drop the pickup

           // armPos(armIdle,0.5);
            movePickUp(autonPickupIdle, 0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 2.0)) {
                myOpMode.telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }


            //Step 6: drive forward to the spike mark

            driveRobot(1, 0.7*FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 0.25)||(parallelEncoder.getCurrentPosition()<(s3Finish+(s5*encTicksToInches))))) {
                myOpMode.telemetry.addData("Path", "Leg 7: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 7: Extract pixel from the pickup
           // armPos(autonArmIdle,0.5);
            driveRobot(0, 0, 0, 0);
            rightWheel.setPower(-0.9);
            //setRightClawPosition(vvHardware.clawOpen);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 2.0)) {
                myOpMode.telemetry.addData("Path", "Leg 6: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            setPickupPower(0,0);
            driveRobot(0, 0, 0, 0);

            myOpMode.telemetry.addData("Path", "Complete");
            myOpMode.telemetry.update();
            myOpMode.sleep(1000);

            break;
        }
    }

    public void autonStageLeft() {
        // Wait for the game to start (driver presses PLAY)
        while (myOpMode.opModeIsActive()) {

            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Step 0; Move pickup up

           /* movePickUp(autonPickupHigh, 0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1.0)) {
                myOpMode.telemetry.addData("Path", "Leg 0: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }*/

            // Step 1:  Drive forward for 2 seconds

            driveRobot(1, FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 1.5)||(parallelEncoder.getCurrentPosition()<(s1Side*encTicksToInches)))) {
                myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 2:  Spin left for 3 seconds

            driveRobot(1, 0, 0, -TURN_SPEED);

            final double s1Finish = parallelEncoder.getCurrentPosition();

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 0.4)||(heading < -s2Turn))) {
                myOpMode.telemetry.addData("Path", "Leg : %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            //Step 3:  Drive Forward for 1 Second

            driveRobot(1, FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 0.4)||(parallelEncoder.getCurrentPosition()<(s1Finish+(s3Forward*encTicksToInches))))) {
                myOpMode.telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            //Step 4:  Drive Backward for 1 Second

            driveRobot(1, -FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 0.4)) {
                myOpMode.telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            // Step 5:  Stop
            driveRobot(0, 0, 0, 0);

            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1.0)) {
                myOpMode.telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            //unhook the pickup
            movePickUp(5,0.5);

            armPos(armStart,0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 2.0)) {
                myOpMode.telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            // Step 6: Drop the pickup

            movePickUp(autonPickupIdle, 0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1.0)) {
                myOpMode.telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            //Step 7:  Drive Forward for 1 Second

            driveRobot(1,0.7* FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 0.6)) {
                myOpMode.telemetry.addData("Path", "Leg 8: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 8: Extract pixel from the pickup

            driveRobot(0, 0, 0, 0);
            rightWheel.setPower(-0.9);
            //setRightClawPosition(vvHardware.clawOpen);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1.5)) {
                myOpMode.telemetry.addData("Path", "Leg 6: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            driveRobot(0, 0, 0, 0);

            myOpMode.telemetry.addData("Path", "Complete");
            myOpMode.telemetry.update();
            myOpMode.sleep(1000);

            break;
        }
    }

    public void autonStageRight() {
        // Wait for the game to start (driver presses PLAY)
        while (myOpMode.opModeIsActive()) {

            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            //Step 0; Move pickup up

           /* movePickUp(autonPickupHigh, 0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1.0)) {
                myOpMode.telemetry.addData("Path", "Leg 0: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }*/

            // Step 1:  Drive forward for 3 seconds

            driveRobot(1, FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1.5)) {
                myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 2:  Spin right for 2 seconds

            driveRobot(1, 0, 0, TURN_SPEED);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 0.4)||(heading > s2Turn))) {
                myOpMode.telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 3:  Drive Forward for 1 Second

            driveRobot(1, FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 0.4)) {
                myOpMode.telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            //Step 4:  Drive Backward for 1 Second

            driveRobot(1, -FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 0.6)) {
                myOpMode.telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 5:  Stop
            driveRobot(0, 0, 0, 0);

            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1.0)) {
                myOpMode.telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            // Step 5.5 unhook the pickup
            movePickUp(5,0.5);

            armPos(armStart,0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 2.0)) {
                myOpMode.telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            // Step 6: Drop the pickup

            movePickUp(autonPickupIdle, 0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1.0)) {
                myOpMode.telemetry.addData("Path", "Leg 6: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            //Step 7:  Drive Forward for 1 Second

            driveRobot(1,0.7* FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 0.25)) {
                myOpMode.telemetry.addData("Path", "Leg 8: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 7: Extract pixel from the pickup

            driveRobot(0, 0, 0, 0);
            rightWheel.setPower(-0.9);
            //setRightClawPosition(vvHardware.clawOpen);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1.5)) {
                myOpMode.telemetry.addData("Path", "Leg 6: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            driveRobot(0, 0, 0, 0);

            myOpMode.telemetry.addData("Path", "Complete");
            myOpMode.telemetry.update();
            myOpMode.sleep(1000);
            break;
        }
    }
    public void autonTopBack () {
        // Wait for the game to start (driver presses PLAY)
        while (myOpMode.opModeIsActive()) {
            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            // Step 1:  Drive forward for 2 seconds or on ticks

            movePickUp(5,0.5);
            armPos(armStart,0.7);
            myOpMode.sleep(1000);
            driveRobot(1, FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 1.9)||(parallelEncoder.getCurrentPosition()<(s1Top*encTicksToInches)))) {
                myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 2:  Spin right for 1.3 seconds

            driveRobot(0.3, 0, 0, 0); //No Turn
            final double s1Finish = parallelEncoder.getCurrentPosition();

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1)) {
                myOpMode.telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 3:  Drive Backward for 0.6 Second

            driveRobot(1, -FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 0.6)||(parallelEncoder.getCurrentPosition()>(s1Finish-(s3Reverse*encTicksToInches))))) {
                myOpMode.telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 4:  Stop
            driveRobot(0, 0, 0, 0);
            final double s3Finish = parallelEncoder.getCurrentPosition();

            while (myOpMode.opModeIsActive() && (runtime.seconds() < 1.0)) {
                myOpMode.telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            // Step 5 unhook the pickup

            movePickUp(5,0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 2.0)) {
                myOpMode.telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            // Step 5.5: Drop the pickup

            // armPos(armIdle,0.5);
            movePickUp(autonPickupIdle, 0.5);
            myOpMode.sleep(100);
            armPos((armStart-5),0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 2.0)) {
                myOpMode.telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            //Step 6: drive forward to the spike mark

            final double s6Finish = parallelEncoder.getCurrentPosition();

            driveRobot(1, 0.7*FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 0.25)||(parallelEncoder.getCurrentPosition()<(s3Finish+(s5*encTicksToInches))))) {
                myOpMode.telemetry.addData("Path", "Leg 7: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            // Step 7: Extract pixel from the pickup
            // armPos(autonArmIdle,0.5);
            driveRobot(0, 0, 0, 0);
            rightWheel.setPower(-0.9);
            //setRightClawPosition(vvHardware.clawOpen);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 2.0)) {
                myOpMode.telemetry.addData("Path", "Leg 7: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
/*
            setPickupPower(0,0);
            driveRobot(0, 0, 0, 0);

            myOpMode.telemetry.addData("Path", "Complete");
            myOpMode.telemetry.update();
            myOpMode.sleep(1000);
*/          //Here is where the Spike mark code ends

            //Step 8 strafe to the backdrop

            setPickupPower(0,0);

            driveRobot(1, 0, -FORWARD_SPEED, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 2.5)||(perpendicularEncoder.getCurrentPosition()<(-(s11+2)*encTicksToInches)))) {
                myOpMode.telemetry.addData("Path", "Leg 8: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            //Step 9: Turn towards the backdrop

            driveRobot(1, 0, 0, -TURN_SPEED);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 0.8)||(heading < -(s9Turn)))) {
                myOpMode.telemetry.addData("Path", "Leg 9: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            // Step 10 arm up
            driveRobot(1, 0, 0, 0);
            final double s10Finish = parallelEncoder.getCurrentPosition();
            armPos(armLow,0.5);
            if(Math.abs(leftArm.getCurrentPosition() - armLow)<10)
                movePickUp(autonPickupIdle,0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 3.0)) {
                myOpMode.telemetry.addData("Path", "Leg 10: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            //Step 11 move closer to the backdrop
            driveRobot(1, FORWARD_SPEED, 0, 0);

            //||(parallelEncoder.getCurrentPosition()>(s10Finish+(4*encTicksToInches)))
            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 0.9))) {
                myOpMode.telemetry.addData("Path", "Leg 11: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            //Step 11.5 strafe to the backdrop center
            driveRobot(0,0,0,0);
            myOpMode.sleep(100);
            final double s11Finish = perpendicularEncoder.getCurrentPosition();
            driveRobot(1, 0, FORWARD_SPEED, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 0.5)||(perpendicularEncoder.getCurrentPosition()>(s11Finish+(6*encTicksToInches))))) {
                myOpMode.telemetry.addData("Path", "Leg 11: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            //Step 12: push the pixel out

            driveRobot(0, 0, 0, 0);
            final double s12Finish = perpendicularEncoder.getCurrentPosition();
            leftWheel.setPower(0.9);
            //setRightClawPosition(vvHardware.clawOpen);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 2.0)) {
                myOpMode.telemetry.addData("Path", "Leg 12: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            //Step 13 strafe to the side
            leftWheel.setPower(0);
            movePickUp(autonPickupHigh,0.7);
            myOpMode.sleep(100);
            driveRobot(1, 0, -FORWARD_SPEED, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 1)||(perpendicularEncoder.getCurrentPosition()<(s12Finish-(18*encTicksToInches))))) {
                myOpMode.telemetry.addData("Path", "Leg 11: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            driveRobot(0, 0, 0, 0);
            movePickUp(autonPickupIdle,0.5);
            armPos(armIdle,0.5);

            myOpMode.telemetry.addData("Path", "Complete");
            myOpMode.telemetry.update();
            myOpMode.sleep(1000);

            break;
        }
    }
    public void autonRightBack () {
        // Wait for the game to start (driver presses PLAY)
        while (myOpMode.opModeIsActive()) {

            final double s6Finish = parallelEncoder.getCurrentPosition();

            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            // Step 1 : turn to original position

            driveRobot(1, 0, 0, TURN_SPEED);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 0.4) || (heading > s2Turn))) {
                myOpMode.telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            //Step 2 strafe to the backdrop

            setPickupPower(0, 0);
            driveRobot(1, 0, -FORWARD_SPEED, 0);
            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 2.0) || (perpendicularEncoder.getCurrentPosition() < (-s11 * encTicksToInches)))) {
                myOpMode.telemetry.addData("Path", "Leg 8: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            //Step 3: Turn towards the backdrop
            imu.resetYaw();
            driveRobot(1, 0, 0, -TURN_SPEED);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 0.4) || (heading < -s9Turn))) {
                myOpMode.telemetry.addData("Path", "Leg 9: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            // Step 4 arm up
            armPos(armLow, 0.5);
            if (Math.abs(leftArm.getCurrentPosition() - armLow) < 10)
                movePickUp(autonPickupIdle, 0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 3.0)) {
                myOpMode.telemetry.addData("Path", "Leg 10: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            //Step 5 move closer to the backdrop
            driveRobot(0.7, FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 1.0) || (parallelEncoder.getCurrentPosition() < (s6Finish + (s12 * encTicksToInches))))) {
                myOpMode.telemetry.addData("Path", "Leg 11: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            //Step 6: push the pixel out

            driveRobot(0, 0, 0, 0);
            leftWheel.setPower(-0.9);
            //setRightClawPosition(vvHardware.clawOpen);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 2.0)) {
                myOpMode.telemetry.addData("Path", "Leg 12: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            leftWheel.setPower(0);
            driveRobot(0, 0, 0, 0);

            myOpMode.telemetry.addData("Path", "Complete");
            myOpMode.telemetry.update();
            myOpMode.sleep(1000);

            break;
        }
    }

    public void autonLeftBack() {

        // Wait for the game to start (driver presses PLAY)
        while (myOpMode.opModeIsActive()) {

            final double s6Finish = parallelEncoder.getCurrentPosition();

            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            //Step 1 strafe to the backdrop

            setPickupPower(0, 0);
            driveRobot(1, 0, -FORWARD_SPEED, 0);
            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 2.0) || (perpendicularEncoder.getCurrentPosition() < (-s11 * encTicksToInches)))) {
                myOpMode.telemetry.addData("Path", "Leg 8: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            //Step 2: Turn towards the backdrop
            imu.resetYaw();
            driveRobot(1, 0, 0, -TURN_SPEED);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 0.4) || (heading < -s9Turn))) {
                myOpMode.telemetry.addData("Path", "Leg 9: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            // Step 3 arm up
            armPos(armLow, 0.5);
            if (Math.abs(leftArm.getCurrentPosition() - armLow) < 10)
                movePickUp(autonPickupIdle, 0.5);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 3.0)) {
                myOpMode.telemetry.addData("Path", "Leg 10: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            //Step 4 move closer to the backdrop
            driveRobot(0.7, FORWARD_SPEED, 0, 0);

            runtime.reset();
            while (myOpMode.opModeIsActive() && ((runtime.seconds() < 1.0) || (parallelEncoder.getCurrentPosition() < (s6Finish + (s12 * encTicksToInches))))) {
                myOpMode.telemetry.addData("Path", "Leg 11: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }

            //Step 5: push the pixel out

            driveRobot(0, 0, 0, 0);
            leftWheel.setPower(-0.9);
            //setRightClawPosition(vvHardware.clawOpen);

            runtime.reset();
            while (myOpMode.opModeIsActive() && (runtime.seconds() < 2.0)) {
                myOpMode.telemetry.addData("Path", "Leg 12: %4.1f S Elapsed", runtime.seconds());
                myOpMode.telemetry.update();
            }
            leftWheel.setPower(0);
            driveRobot(0, 0, 0, 0);

            myOpMode.telemetry.addData("Path", "Complete");
            myOpMode.telemetry.update();
            myOpMode.sleep(1000);

            break;
        }
    }
}



