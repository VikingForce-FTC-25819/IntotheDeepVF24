package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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


        Trajectory prepareForPickUpSpikeThree = drive.trajectoryBuilder(trajectoryScore.end())
                .lineToLinearHeading(new Pose2d(-52, -57, Math.toRadians(90)))
                .build();

        Trajectory pickUpSpikeThree = drive.trajectoryBuilder(prepareForPickUpSpikeThree.end())
                //.lineToLinearHeading(new Pose2d(-57, -44, Math.toRadians(90)))
                .forward(4,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        Trajectory trajectoryScoreFromSpikeThree = drive.trajectoryBuilder(pickUpSpikeThree.end())
                .lineToLinearHeading(new Pose2d(-57, -56, Math.toRadians(225)))
                .build();

        Trajectory prepareForPickUpSpikeTwo = drive.trajectoryBuilder(trajectoryScoreFromSpikeThree.end())
                .lineToLinearHeading(new Pose2d(-61, -57, Math.toRadians(90)))
                .build();

        Trajectory pickUpSpikeTwo = drive.trajectoryBuilder(prepareForPickUpSpikeTwo.end())
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        Trajectory trajectoryScoreFromSpikeTwo = drive.trajectoryBuilder(pickUpSpikeTwo.end())
                .lineToLinearHeading(new Pose2d(-57, -56
                        , Math.toRadians(225)))
                .build();

        Trajectory prepareForPickUpSpikeOne = drive.trajectoryBuilder(trajectoryScoreFromSpikeTwo.end())
                .lineToLinearHeading(new Pose2d(-48, -50, Math.toRadians(130)))
                .build();

        Trajectory pickUpSpikeOne = drive.trajectoryBuilder(prepareForPickUpSpikeOne.end())
                .lineToLinearHeading(new Pose2d(-56, -43, Math.toRadians(130)),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        Trajectory backUpFromSpikeOne = drive.trajectoryBuilder(pickUpSpikeOne.end())
                //.lineToLinearHeading(new Pose2d(-54, -40, Math.toRadians(129)),
                        .back(8,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        Trajectory trajectoryScoreFromSpikeOne = drive.trajectoryBuilder(backUpFromSpikeOne.end())
                .lineToLinearHeading(new Pose2d(-57, -56
                        , Math.toRadians(225)))
                .build();

        Trajectory rotateForPark = drive.trajectoryBuilder(trajectoryScoreFromSpikeOne.end())
                .lineToLinearHeading(new Pose2d(-57, -50, Math.toRadians(90)))
                .build();

        Trajectory parkInAscentZone = drive.trajectoryBuilder(rotateForPark.end())
                .splineTo(new Vector2d(-30,-10), Math.toRadians(0))
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
        // move to observation zone
        drive.followTrajectory(rotateForPark);
        robot.storeRobot();
        drive.followTrajectory(parkInAscentZone);

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
