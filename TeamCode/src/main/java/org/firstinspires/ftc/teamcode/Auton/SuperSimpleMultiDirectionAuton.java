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

import org.firstinspires.ftc.teamcode.Core.VfHardware;

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

@Autonomous(name="Super Simple Multi-Direction Auton", group="Concept")
public class SuperSimpleMultiDirectionAuton extends LinearOpMode {

    //public static final int FORWARD_DISTANCE_INCHES = 98;
    public static final int FORWARD_DISTANCE_INCHES = 13;

    public static final int STRAFE_DISTANCE = 12;
    // Starting at edge of tile closest to net zone
    public static final int BACKWARD_DISTANCE_INCHES = 36;
    // Starting at edge of second tile closest to net zone
    // public static final int BACKWARD_DISTANCE_INCHES = 34;
    // Starting at edge of third tile closest to net zone
    //public static final int BACKWARD_DISTANCE_INCHES = 58;
    // Starting at edge of fourth tile closest to net zone
    //public static final int BACKWARD_DISTANCE_INCHES = 13;
    private static final double INITIAL_PAUSE_SECONDS = 1.0;
    private static final double TRANSITION_PAUSE_SECONDS = 1.0;

    static final double     SPEED = 0.3;

    @Override
    public void runOpMode() {
        VfHardware robot = new VfHardware(this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Not ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Wait for INITIAL_PAUSE_SECONDS of time before doing anything
        robot.pause(INITIAL_PAUSE_SECONDS);

        robot.autoDrive(STRAFE_DISTANCE, SPEED, AutonDirection.right);

        robot.pause(TRANSITION_PAUSE_SECONDS);

        robot.autoDrive(FORWARD_DISTANCE_INCHES, SPEED, AutonDirection.forward);

        robot.pause(TRANSITION_PAUSE_SECONDS);

        robot.autoDrive(BACKWARD_DISTANCE_INCHES, SPEED, AutonDirection.reverse);

        robot.pause(TRANSITION_PAUSE_SECONDS);

        robot.autoDrive(STRAFE_DISTANCE, SPEED, AutonDirection.left);

        robot.stop();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}

