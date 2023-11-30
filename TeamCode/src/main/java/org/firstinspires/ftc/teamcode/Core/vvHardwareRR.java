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
    final public int armStart = 10;
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
        //rightClaw = myOpMode.hardwareMap.get(Servo.class,"RSW");
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
     * Extract the purple pixel
     *
     * @param RWTime
     */
    public void extractPurple(double RWTime) {
        rightWheel.setPower(-0.9);
        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < RWTime)) {
            rightWheel.setPower(0);
        }
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
}


