package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.vvHardware;
import org.firstinspires.ftc.teamcode.Core.vvHardwareRR;
import org.firstinspires.ftc.teamcode.Core.vvRoadRunnerDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * VV OpMode driving a path by RoadRunner
 *
 */

@Autonomous(name="AutoRR", group="Concept")

public class vvAutoDriveByRR extends LinearOpMode {
    /**This needs to be replaced with usage of our vvHardware class and create a new method to drive by encoder
     * need to create the appropriate variables and structure this class per your pseudo code
     */
    /* Declare OpMode members. */
    vvHardwareRR   robot       = new vvHardwareRR(this);
    private ElapsedTime     runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        robot.init();

        vvRoadRunnerDrive vvdrive = new vvRoadRunnerDrive(hardwareMap);

        double armEPower = 0.8;
        double pickUpPwr = 0.7;
        final int autonPickupIdle = -30; // the idle position for the pickup motor 109
        final int autonPickupHigh = -5; // the placing position for the pickup motor in the high position 148
        final int autonPickupLow = -27; // the placing position for the pickup motor in the low/forward position 5

        // the amount of time the pickup takes to activate in seconds
        final double pickupTime;
        // the amount of time the arm takes to raise in seconds
        final double armTime;

        final int armIdle = 0; // -84
        final int armLow = 110; // the low encoder position for the arm -23
        final int armHigh = 401; // the high-overhead encoder position for the arm 329
        final int armHang = 470;

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // We want to start the bot at x: 14, y: -60, heading: 90 degrees
        Pose2d startPose = new Pose2d(14, -60, Math.toRadians(90));

        vvdrive.setPoseEstimate(startPose);

        TrajectorySequence purpleDropTop = vvdrive.trajectorySequenceBuilder(startPose)
                .forward(24)
                .back(4)
                .waitSeconds(1)
                .build();

        TrajectorySequence yellowDropTop = vvdrive.trajectorySequenceBuilder(purpleDropTop.end())
                .turn(Math.toRadians(-90))
                .forward(48)
                .build();

        /*Trajectory traj1 = vvdrive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(14, -36), Math.toRadians(90))
                .build();

        Trajectory traj2 = vvdrive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(28, 14), Math.toRadians(45))
                .build();

         */
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("Running...");
        telemetry.update();
        vvdrive.followTrajectorySequence(purpleDropTop);
        robot.extractPurple(1);
        sleep(1000);
        robot.armPos(armLow,armEPower); //Drive location
        robot.movePickUp(autonPickupLow,pickUpPwr);
        sleep(1000);
        vvdrive.followTrajectorySequence(yellowDropTop);

    }
}
