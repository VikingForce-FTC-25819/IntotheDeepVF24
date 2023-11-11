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

/*Magnus hardware class
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 */

@Disabled
public class vvHardwareMag {


    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor rightRear;
    public DcMotor leftRear;
    public DcMotor leftArm;
    public DcMotor rightArm;
    public DcMotor pickUp;
    public DcMotor lift;
    public CRServo rightWheel;
    public CRServo leftWheel;
    public Servo drone;

    public IMU imu;
    public DcMotor parallelEncoder;
    public DcMotor perpendicularEncoder;
    public ColorSensor colorSensor;
    public DistanceSensor distFront;
    public DistanceSensor distRear;


    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double droneSet = 0.25;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public vvHardwareMag(LinearOpMode opmode) {
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
        leftFront = myOpMode.hardwareMap.get(DcMotor.class, "FLM");
        rightFront = myOpMode.hardwareMap.get(DcMotor.class, "FRM");
        rightRear = myOpMode.hardwareMap.get(DcMotor.class, "RRM");
        leftRear = myOpMode.hardwareMap.get(DcMotor.class, "RLM");
        //leftArm = myOpMode.hardwareMap.get(DcMotor.class, "armL");
        //rightArm = myOpMode.hardwareMap.get(DcMotor.class, "armR");
        //pickUp = myOpMode.hardwareMap.get(DcMotor.class, "pickUp");
        //lift = myOpMode.hardwareMap.get(DcMotorEx.class, "lift");

        //Shadow the motors with encoder-odometry
        parallelEncoder = leftFront;
        perpendicularEncoder = rightFront;

        // Define Servos
        //rightWheel = myOpMode.hardwareMap.crservo.get("RSW");
        //leftWheel = myOpMode.hardwareMap.crservo.get("LSW");
        //drone = myOpMode.hardwareMap.get(Servo.class,"drone");

        //drone.scaleRange(0,1);
        //drone.setDirection(Servo.Direction.REVERSE);
        //drone.setPosition(droneSet);

        //define and initialize sensors
        //colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "CLR");

        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
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
        //rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
        //leftArm.setDirection(DcMotor.Direction.REVERSE);
        //lift.setDirection(DcMotor.Direction.REVERSE);
/*
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pickUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
*/
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //pickUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //pickUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        //myOpMode.telemetry.addData("Drone Servo", drone.getPosition());
        myOpMode.telemetry.update();
    }
    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial-X motion), Strafe (Side-to-Side-Y motion) and Turn (Yaw-Z motion).
     * Then sends these power levels to the motors.
     * @param drivePower Global variable to set total drive power
     * @param driveY    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param strafe    Side to side driving power (-1.0 to 1.0) +ve is left
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double drivePower, double driveY, double strafe, double turn) {
        double strafeBias = 0.9; // given weight distribution need to bias the back wheel power
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
        lift.setPower(0.95);
    }
}
