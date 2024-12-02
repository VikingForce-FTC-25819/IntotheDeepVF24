package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 */
@Config
@Autonomous(group = "1")
public class ScoreLotsInHighBasket extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(-16, -63, Math.toRadians(90));

        drive.setPoseEstimate(start);

        Trajectory moveOffWall = drive.trajectoryBuilder(start)
                .forward(2)
                .build();

        Trajectory trajectoryScore = drive.trajectoryBuilder(moveOffWall.end())
                .lineToLinearHeading(new Pose2d(-57, -56, Math.toRadians(225)))
                .build();

        Trajectory backUpFromBasket = drive.trajectoryBuilder(trajectoryScore.end())
                .back(20)
                .build();

        Trajectory prepareForPickUpSpikeThree = drive.trajectoryBuilder(backUpFromBasket.end())
                .lineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(90)))
                .build();

        Trajectory pickUpSpikeThree = drive.trajectoryBuilder(prepareForPickUpSpikeThree.end())
                //.lineToLinearHeading(new Pose2d(-57, -44, Math.toRadians(90)))
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        Trajectory trajectoryScoreFromSpikeThree = drive.trajectoryBuilder(pickUpSpikeThree.end())
                .lineToLinearHeading(new Pose2d(-57, -56, Math.toRadians(225)))
                .build();

        Trajectory prepareForPickUpSpikeTwo = drive.trajectoryBuilder(backUpFromBasket.end())
                .lineToLinearHeading(new Pose2d(-59, -50, Math.toRadians(90)))
                .build();

        Trajectory pickUpSpikeTwo = drive.trajectoryBuilder(prepareForPickUpSpikeTwo.end())
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        Trajectory trajectoryScoreFromSpikeTwo = drive.trajectoryBuilder(pickUpSpikeTwo.end())
                .lineToLinearHeading(new Pose2d(-57, -56
                        , Math.toRadians(225)))
                .build();

        Trajectory prepareForPickUpSpikeOne = drive.trajectoryBuilder(backUpFromBasket.end())
                .lineToLinearHeading(new Pose2d(-43, -43, Math.toRadians(150)))
                .build();

        Trajectory pickUpSpikeOne = drive.trajectoryBuilder(prepareForPickUpSpikeTwo.end())
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        Trajectory trajectoryScoreFromSpikeOne = drive.trajectoryBuilder(pickUpSpikeOne.end())
                .lineToLinearHeading(new Pose2d(-57, -56
                        , Math.toRadians(225)))
                .build();

        Trajectory parkInObservation = drive.trajectoryBuilder(backUpFromBasket.end())
                .lineToLinearHeading(new Pose2d(48, -60, Math.toRadians(90)))
                .build();


        waitForStart();
        // score pre-loaded sample
        drive.followTrajectory(moveOffWall);
        robot.raiseForHighBasket();
        drive.followTrajectory(trajectoryScore);
        safelyScoreSample(robot);
        // score spike three
        backupSafely(drive, backUpFromBasket);
        safelyCollectSample(robot);
        drive.followTrajectory(prepareForPickUpSpikeThree);
        drive.followTrajectory(pickUpSpikeThree);
        collectSampleWithClaw(robot);
        drive.followTrajectory(trajectoryScoreFromSpikeThree);
        safelyScoreSample(robot);
        // score spike two
        backupSafely(drive, backUpFromBasket);
        safelyCollectSample(robot);
        drive.followTrajectory(prepareForPickUpSpikeTwo);
        drive.followTrajectory(pickUpSpikeTwo);
        collectSampleWithClaw(robot);
        drive.followTrajectory(trajectoryScoreFromSpikeTwo);
        safelyScoreSample(robot);
        // score spike one
        backupSafely(drive, backUpFromBasket);
        safelyCollectSample(robot);
        drive.followTrajectory(prepareForPickUpSpikeOne);
        drive.followTrajectory(pickUpSpikeOne);
        collectSampleWithClaw(robot);
        drive.followTrajectory(trajectoryScoreFromSpikeOne);
        safelyScoreSample(robot);
        // move to observation zone
        backupSafely(drive, backUpFromBasket);
        robot.storeRobot();
        drive.followTrajectory(parkInObservation);

        telemetry.update();


    }

    private void safelyCollectSample(Robot robot) {
        runtime.reset();
        while (runtime.seconds() <= .3) {
            robot.collectSample();
        }
    }

    private void safelyScoreSample(Robot robot) {
        runtime.reset();
        while (runtime.seconds() <= .3) {
            robot.deposit();
        }
    }

    private void backupSafely(SampleMecanumDrive drive, Trajectory backUpFromBasket) {
        drive.followTrajectory(backUpFromBasket);
        runtime.reset();
        while (runtime.seconds() <= .5) {
            // give time to backup
        }
    }

    private void collectSampleWithClaw(Robot robot) {
        runtime.reset();
//        while (runtime.seconds() <= 1.5) {
//            robot.collectSample();
//        }
        runtime.reset();
        while (runtime.seconds() <= 0.5) {
            robot.closeClaw();
        }
        runtime.reset();
        while (runtime.seconds() <= 1.5) {
            robot.raiseForHighBasket();
        }
    }

    private static void waitForIt() {
        long startTime = System.currentTimeMillis();
        long duration = 3000; // 5 seconds in milliseconds

        while (System.currentTimeMillis() - startTime < duration) {
            //do nothing to let the deposit keep going....
        }
    }
}
