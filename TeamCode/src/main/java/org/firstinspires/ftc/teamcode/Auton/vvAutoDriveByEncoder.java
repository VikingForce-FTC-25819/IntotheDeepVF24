package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Core.vvHardware;

/*
 * VV OpMode driving a path based on encoder counts
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and eject the pixel
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 */

@Autonomous(name="AutoRed", group="Red")
@Disabled
public class vvAutoDriveByEncoder extends LinearOpMode {
    /**This needs to be replaced with usage of our vvHardware class and create a new method to drive by encoder
     * need to create the appropriate variables and structure this class per your pseudo code
     */
    /* Declare OpMode members. */
    vvHardware   robot       = new vvHardware(this);

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_ODO_REV    = 2000 ;    // GoBILDA Odo pod
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing
    static final double     WHEEL_DIAMETER_INCHES   = 1.89 ;     // Odo diameter, 48mm
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_ODO_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.4;
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    @Override
    public void runOpMode() throws InterruptedException {

        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        int x = 0;
        int y = 0;
        double RWPower = 0;
        double LWPower = 0;
        double RWPowerPU = 0;
        double LWPowerPU = 0;
        double drivePower = 0.5; //global drive power level
        double armPower = 0;
        double armEPower = 0;
        double droneSet = 0.25;
        double droneLaunch = 0;
        double servpos = 0;

        robot.init();

        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  0,  24, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, 0, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 0, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double xInches, double yInches,
                             double timeoutS) {
        int newXTarget; //formerly left
        int newYTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newXTarget = robot.leftFront.getCurrentPosition() + (int)(xInches * COUNTS_PER_INCH);
            newYTarget = robot.rightFront.getCurrentPosition() + (int)(yInches * COUNTS_PER_INCH);
            //leftDrive.setTargetPosition(newLeftTarget);
            //rightDrive.setTargetPosition(newRightTarget);
            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            // Turn On RUN_TO_POSITION
            //leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            double  rangeError      = newYTarget;
            double  headingError    = 0;
            double  yawError        = newYTarget;

            /*
             * Move robot according to desired axes motions
             * Positive X is forward
             * Positive Y is strafe left
             * Positive Yaw is counter-clockwise
             */
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double edriveY  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double eturn   = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            double estrafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            // reset the timeout time and start motion.
            runtime.reset();
            //leftDrive.setPower(Math.abs(speed));
            //rightDrive.setPower(Math.abs(speed));

            //Run until odo is >...
            if (newXTarget > 0 || newXTarget > 0)
                robot.driveAuton(DRIVE_SPEED,edriveY,estrafe,eturn);



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newYTarget,  newXTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        robot.leftFront.getCurrentPosition(), robot.rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            //leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    /*public void odometry() {
    oldRightPosition = currentRightPosition;
    oldLeftPosition = currentLeftPosition;
    oldAuxPosition = currentAuxPosition;

    currentRightPosition = -encoderRight.getCurrentPosition();
    currentLeftPosition = encoderLeft.getCurrentPosition();
    currentAuxPosition = encoderAux.getCurrentPosition();

    int dn1 = currentLeftPosition  - oldLeftPosition;
    int dn2 = currentRightPosition - oldRightPosition;
    int dn3 = currentAuxPosition - oldAuxPosition;

    // the robot has moved and turned a tiny bit between two measurements:
    double dtheta = cm_per_tick * ((dn2-dn1) / (LENGTH));
    double dx = cm_per_tick * ((dn1+dn2) / 2.0);
    double dy = cm_per_tick * (dn3 + ((dn2-dn1) / 2.0));

    telemetrydx = dx;
    telemetrydy = dy;
    telemetrydh = dtheta;

    // small movement of the robot gets added to the field coordinate system:
    pos.h += dtheta / 2;
    pos.x += dx * Math.cos(pos.h) - dy * Math.sin(pos.h);
    pos.y += dx * Math.sin(pos.h) + dy * Math.cos(pos.h);
    pos.h += dtheta / 2;
    pos.h = normDiff(pos.h);
*/
}
