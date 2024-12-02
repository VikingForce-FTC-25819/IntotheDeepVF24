package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 */
@Config
@Autonomous(group = "1")
public class ScoreLotsInHighBasketFaster extends LinearOpMode {

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

        Trajectory prepareForPickUpSpikeThree = drive.trajectoryBuilder(trajectoryScore.end())
                .lineToLinearHeading(new Pose2d(-31, -46, Math.toRadians(140)))
                .build();

        Trajectory pickUpSpikeThree = drive.trajectoryBuilder(prepareForPickUpSpikeThree.end())
                //.lineToLinearHeading(new Pose2d(-57, -44, Math.toRadians(90)))
                .forward(7,
                        getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        getAccelerationConstraint(40))
                .build();

        Trajectory trajectoryScoreFromSpikeThree = drive.trajectoryBuilder(pickUpSpikeThree.end())
                .lineToLinearHeading(new Pose2d(-57, -56, Math.toRadians(225)))
                .build();

        Trajectory prepareForPickUpSpikeTwo = drive.trajectoryBuilder(trajectoryScoreFromSpikeThree.end())
                .lineToLinearHeading(new Pose2d(-41, -43, Math.toRadians(140)))
                .build();

        Trajectory pickUpSpikeTwo = drive.trajectoryBuilder(prepareForPickUpSpikeTwo.end())
                .forward(7,
                        getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        getAccelerationConstraint(40))
                .build();

        Trajectory trajectoryScoreFromSpikeTwo = drive.trajectoryBuilder(pickUpSpikeTwo.end())
                .lineToLinearHeading(new Pose2d(-57, -56
                        , Math.toRadians(225)))
                .build();

        Trajectory prepareForPickUpSpikeOne = drive.trajectoryBuilder(trajectoryScoreFromSpikeTwo.end())
                .lineToLinearHeading(new Pose2d(-44, -35, Math.toRadians(165)))
                .build();

        Trajectory pickUpSpikeOne = drive.trajectoryBuilder(prepareForPickUpSpikeOne.end())
                .forward(8,
                        getVelocityConstraint(40, MAX_ANG_VEL, TRACK_WIDTH),
                        getAccelerationConstraint(40))
                .build();

        Trajectory backUpFromSpikeOne = drive.trajectoryBuilder(pickUpSpikeOne.end())
                .back(10)
                .build();

        Trajectory trajectoryScoreFromSpikeOne = drive.trajectoryBuilder(backUpFromSpikeOne.end())
                .lineToLinearHeading(new Pose2d(-57, -56
                        , Math.toRadians(225)))
                .build();

        Trajectory rotateForPark = drive.trajectoryBuilder(trajectoryScoreFromSpikeOne.end())
                .lineToLinearHeading(new Pose2d(-57, -50, Math.toRadians(90)))
                .build();

//        Trajectory parkInAscentZone = drive.trajectoryBuilder(rotateForPark.end())
//                .splineTo(new Vector2d(-24, -9), Math.toRadians(0))
//                .build();

        Trajectory parkInObservation = drive.trajectoryBuilder(rotateForPark.end())
                .lineToLinearHeading(new Pose2d(48, -60, Math.toRadians(90)))
                .build();


        waitForStart();
        // score pre-loaded sample
        drive.followTrajectory(moveOffWall);
        robot.raiseForHighBasket();
        drive.followTrajectory(trajectoryScore);
        safelyScoreSample(robot);
        // score spike three
        drive.followTrajectory(prepareForPickUpSpikeThree);
        safelyCollectSample(robot);
        drive.followTrajectory(pickUpSpikeThree);
        collectSampleAndRaise(robot);
        drive.followTrajectory(trajectoryScoreFromSpikeThree);
        safelyScoreSample(robot);
        // score spike two
        drive.followTrajectory(prepareForPickUpSpikeTwo);
        safelyCollectSample(robot);
        drive.followTrajectory(pickUpSpikeTwo);
        collectSampleAndRaise(robot);
        drive.followTrajectory(trajectoryScoreFromSpikeTwo);
        safelyScoreSample(robot);
        // score spike one
        drive.followTrajectory(prepareForPickUpSpikeOne);
        safelyCollectSample(robot);
        drive.followTrajectory(pickUpSpikeOne);
        collectSampleOnly(robot);
        drive.followTrajectory(backUpFromSpikeOne);
        raiseClawForHighBasket(robot);
        drive.followTrajectory(trajectoryScoreFromSpikeOne);
        safelyScoreSample(robot);
        // move to park
        drive.followTrajectory(rotateForPark);
        robot.storeRobot();
        drive.followTrajectory(parkInObservation);

        telemetry.update();


    }

    private void safelyCollectSample(Robot robot) {
        runtime.reset();
        while (runtime.seconds() <= .75) {
            robot.collectSample();
        }
    }

    private void safelyScoreSample(Robot robot) {
        runtime.reset();
        while (runtime.seconds() <= .4) {
            // slow down
        }
        runtime.reset();
        while (runtime.seconds() <= .3) {
            robot.deposit();
        }
        robot.raiseArmForBasketExit();
    }

    private void collectSampleAndRaise(Robot robot) {
        collectSampleOnly(robot);
        raiseClawForHighBasket(robot);
    }

    private void collectSampleOnly(Robot robot) {
        runtime.reset();
        while (runtime.seconds() <= 0.3) {
            robot.closeClaw();
        }
    }

    private void raiseClawForHighBasket(Robot robot) {
        runtime.reset();
        while (runtime.seconds() <= .75) {
            robot.raiseForHighBasket();
        }
    }
}
