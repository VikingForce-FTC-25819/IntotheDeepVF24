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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 */
@Config
@Autonomous(group = "1")
public class SpecimenHangRightSide extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(16, -63, Math.toRadians(90));

        drive.setPoseEstimate(start);

        Trajectory moveOffWall = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(16, -58, Math.toRadians(90)))
                .build();

        Trajectory alignToScore = drive.trajectoryBuilder(moveOffWall.end())
                .lineToLinearHeading(new Pose2d(8, -58, Math.toRadians(90)))
                .build();

        Trajectory trajectoryScore = drive.trajectoryBuilder(alignToScore.end())
                .lineToLinearHeading(new Pose2d(8, -36, Math.toRadians(90)))
                .build();

        Trajectory lockItIn = drive.trajectoryBuilder(trajectoryScore.end())
                .forward(1,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        Trajectory backUpFromHang = drive.trajectoryBuilder(lockItIn.end())
                .lineToLinearHeading(new Pose2d(8, -42, Math.toRadians(90)),
                        getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH),
                        getAccelerationConstraint(35))
                .build();

        Trajectory strafeRightClearOfSubmersible = drive.trajectoryBuilder(backUpFromHang.end())
                .lineToLinearHeading(new Pose2d(38, -42, Math.toRadians(90)),
                        getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH),
                        getAccelerationConstraint(35))
                .build();

        Trajectory forwardOfSpikeMarks = drive.trajectoryBuilder(strafeRightClearOfSubmersible.end())
                .lineToLinearHeading(new Pose2d(38, -9, Math.toRadians(90)))
                .build();

        Trajectory strafeToSpikeThree = drive.trajectoryBuilder(forwardOfSpikeMarks.end())
                .lineToLinearHeading(new Pose2d(46, -9, Math.toRadians(90)))
                .build();


        Trajectory pushToObservationZoneSpikeThree = drive.trajectoryBuilder(strafeToSpikeThree.end())
                .back(48,
                    getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH),
                    getAccelerationConstraint(35))
                .build();

        Trajectory lineUpForSpecimenPickUp = drive.trajectoryBuilder(pushToObservationZoneSpikeThree.end())
                .lineToLinearHeading(new Pose2d(48, -43, Math.toRadians(270)))
                .build();

        Trajectory pickUpSpecimenOne = drive.trajectoryBuilder(lineUpForSpecimenPickUp.end())
                .lineToLinearHeading(new Pose2d(48, -54, Math.toRadians(270)),
                        getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH),
                        getAccelerationConstraint(35))
                .build();

        Trajectory alignToScoreFromFirstPickUp = drive.trajectoryBuilder(pickUpSpecimenOne.end())
                .lineToLinearHeading(new Pose2d(-8, -58, Math.toRadians(90)),
                        getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH),
                        getAccelerationConstraint(35))
                .build();

        Trajectory trajectoryScoreFromFirstPickUp = drive.trajectoryBuilder(alignToScoreFromFirstPickUp.end())
                .lineToLinearHeading(new Pose2d(-8, -36, Math.toRadians(90)))
                .build();

        Trajectory lockItInSecondSpecimen = drive.trajectoryBuilder(trajectoryScoreFromFirstPickUp.end())
                .forward(1,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        Trajectory backUpFromHangSecondSpecimen = drive.trajectoryBuilder(lockItInSecondSpecimen.end())
                .lineToLinearHeading(new Pose2d(-8, -42, Math.toRadians(90)),
                        getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH),
                        getAccelerationConstraint(35))
                .build();

        Trajectory parkInObservation = drive.trajectoryBuilder(backUpFromHangSecondSpecimen.end())
                .lineToLinearHeading(new Pose2d(63, -60, Math.toRadians(90)))
                .build();
        waitForStart();
        drive.followTrajectory(moveOffWall);
        safelyRaiseArmForHang(robot);
        telemetry.update();
        drive.followTrajectory(alignToScore);
        drive.followTrajectory(trajectoryScore);
        safelyDepositSpecimen(robot, drive, lockItIn);
        telemetry.update();
        drive.followTrajectory(backUpFromHang);
        drive.followTrajectory(strafeRightClearOfSubmersible);
        drive.followTrajectory(forwardOfSpikeMarks);
        drive.followTrajectory(pushToObservationZoneSpikeThree);
        drive.followTrajectory(lineUpForSpecimenPickUp);
        robot.raiseForSpecimenCollect();
        telemetry.update();
        safelyApproachSpecimen(drive, pickUpSpecimenOne);
        collectSpecimenWithClaw(robot);
        telemetry.update();
        drive.followTrajectory(alignToScoreFromFirstPickUp);
        drive.followTrajectory(trajectoryScoreFromFirstPickUp);
        safelyDepositSpecimen(robot, drive, lockItInSecondSpecimen);
        drive.followTrajectory(backUpFromHangSecondSpecimen);
        safelyStoreRobot(robot);
        drive.followTrajectory(parkInObservation);
        telemetry.update();


    }

    private void safelyStoreRobot(Robot robot) {
        runtime.reset();
        while (runtime.seconds() <= 0.5) {
            // wait to store robot so claw is safe
        }
        robot.storeRobot();
    }

    private void safelyApproachSpecimen(SampleMecanumDrive drive, Trajectory trajectory) {
        runtime.reset();
        while (runtime.seconds() <= 0.75) {
            // wait for human player
        }
        drive.followTrajectory(trajectory);
    }

    private void safelyRaiseArmForHang(Robot robot) {
        runtime.reset();
        while (runtime.seconds() <= .3) {
            robot.raiseForSpecimenHang();
        }
    }

    private void safelyDepositSpecimen(Robot robot, SampleMecanumDrive drive, Trajectory lockItIn) {
        robot.adjustArmAngle(-0.5);
        runtime.reset();
        while (runtime.seconds() <= 0.3) {
            // be patient
        }
        drive.followTrajectory(lockItIn);
        runtime.reset();
        while (runtime.seconds() <= .3) {
            robot.deposit();
        }
    }

    private void collectSpecimenWithClaw(Robot robot) {
        runtime.reset();
        while (runtime.seconds() <= 1) {
            robot.closeClaw();
        }
        safelyRaiseArmForHang(robot);
    }

    private static void waitForIt() {
        long startTime = System.currentTimeMillis();
        long duration = 3000; // 5 seconds in milliseconds

        while (System.currentTimeMillis() - startTime < duration) {
            //do nothing to let the deposit keep going....
        }
    }
}
