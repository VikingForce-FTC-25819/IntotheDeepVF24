/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This is a simple OpMode to understand how the odometry pods can be used to navigate based on
 * distance traveled.  We can use this OpMode to score a potential of 5 points.  2 by pushing a
 * preloaded sample to the net zone, then 3 by backing up to the observation zone.
 *
 * The OpMode is designed to
 * 1. Wait for a period of time defined by INITIAL_PAUSE_SECONDS
 * 2. Accelerate forward for a distance of FORWARD_DISTANCE_INCHES
 * 3. Stop all the motors
 * 4. Wait for a period of time defined by TRANSITION_PAUSE_SECONDS
 * 5. Accelerate backward for a distance of BACKWARD_DISTANCE_INCHES
 * 6. Stop all the motors
 *
 */

@Autonomous(name="VF Simple Auton - Coach", group="1")
public class HelloOdometryBasedAutonomousOpMode extends LinearOpMode {

    public static final int FORWARD_DISTANCE_INCHES = 36;
    public static final int BACKWARD_DISTANCE_INCHES = 120;
    private static final double INITIAL_PAUSE_SECONDS = 3.0;
    private static final double TRANSITION_PAUSE_SECONDS = 3.0;
    private DcMotor         backLeft   = null;
    private DcMotor         backRight  = null;
    private DcMotor         frontLeft   = null;
    private DcMotor         frontRight  = null;
    private DcMotor         parallelSensor = null;
    private DcMotor         perpendicularSensor = null;
    private ElapsedTime     runtime = new ElapsedTime();
    static final double     FORWARD_SPEED = 0.3;
    static final double     TURN_SPEED    = 0.5;
    public static double WHEEL_DIAMETER = 1.88976; // in
    public static final double TICKS_PER_REV = 2000;
    public double encTicksPerInches = TICKS_PER_REV/(WHEEL_DIAMETER*Math.PI);
    public double encInchesPerTicks = (WHEEL_DIAMETER*Math.PI)/TICKS_PER_REV;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLeft = hardwareMap.get(DcMotor.class, "RLM");
        backRight = hardwareMap.get(DcMotor.class, "RRM");
        frontLeft = hardwareMap.get(DcMotor.class, "FLM");
        frontRight = hardwareMap.get(DcMotor.class, "FRM");
        parallelSensor = frontLeft;

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // because we have four motors we need set all 4 motor directions
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection((DcMotorSimple.Direction.REVERSE));
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Wait for INITIAL_PAUSE_SECONDS of time before doing anything
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < INITIAL_PAUSE_SECONDS) {
            telemetry.addData("Initial Pause", "Paused: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Drive Forward for FORWARD_DISTANCE_INCHES
        parallelSensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelSensor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // these 2 lines needed to zero out the odometry sensor
        frontLeft.setPower(FORWARD_SPEED);
        frontRight.setPower(FORWARD_SPEED);
        backLeft.setPower(FORWARD_SPEED);
        backRight.setPower(FORWARD_SPEED);
        // reset our stopwatch
        runtime.reset();
        // our odometry pod is reversed in orientation so going forward gives a negative value
        // this is why we need to use a negative sign in our comparison
        // we want to drive forwards until our current position is larger than our desired forward distance
        // so we drive until our current position is no longer <= our desired distance
        while (opModeIsActive() && -parallelSensor.getCurrentPosition() <= FORWARD_DISTANCE_INCHES * encTicksPerInches) {
            telemetry.addData("Distance", "Forward: %4.1f Inches", -parallelSensor.getCurrentPosition()/encInchesPerTicks);
            telemetry.addData("Path", "%4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Stop
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Wait for TRANSITION_PAUSE_SECONDS of time before doing anything
        // reset our stopwatch
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < TRANSITION_PAUSE_SECONDS) {
            telemetry.addData("Transition Pause", "Paused: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Drive Backward for BACKWARD_DISTANCE_INCHES
        parallelSensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelSensor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // need to zero out the odometry pod for new calculations
        // use negative speed to go backward
        frontRight.setPower(-FORWARD_SPEED);
        frontLeft.setPower(-FORWARD_SPEED);
        backLeft.setPower(-FORWARD_SPEED);
        backRight.setPower(-FORWARD_SPEED);

        // reset timer
        runtime.reset();
        // our odometry pod is reversed in orientation so going backwards gives a positive value
        // we want to drive backwards until our current position is larger than our desired backward distance
        // so we drive until our current position is no longer <= our desired distance
        while (opModeIsActive() && parallelSensor.getCurrentPosition() <= BACKWARD_DISTANCE_INCHES * encTicksPerInches) {
            telemetry.addData("Distance", "Backward: %4.1f Inches", -parallelSensor.getCurrentPosition()/encInchesPerTicks);
            telemetry.addData("Path", "%4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.addData("Current position", parallelSensor.getCurrentPosition());
        telemetry.update();
        sleep(1000);
    }
}

