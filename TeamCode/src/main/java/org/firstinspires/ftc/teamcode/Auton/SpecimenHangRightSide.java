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
public class SpecimenHangRightSide extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(16, -63, Math.toRadians(90));

        drive.setPoseEstimate(start);

        Trajectory moveOffWall = drive.trajectoryBuilder(start)
                .forward(5)
                .build();

        Trajectory alignToScore = drive.trajectoryBuilder(moveOffWall.end())
                .lineToLinearHeading(new Pose2d(8, -58, Math.toRadians(90)))
                .build();

        Trajectory trajectoryScore = drive.trajectoryBuilder(alignToScore.end())
                .forward(22)
                .build();

        Trajectory lockItIn = drive.trajectoryBuilder(trajectoryScore.end())
                .forward(2,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        Trajectory backUpFromHang = drive.trajectoryBuilder(lockItIn.end())
                .back(5)
                .build();

        Trajectory strafeRightClearOfSubmersible = drive.trajectoryBuilder(backUpFromHang.end())
                .strafeRight(26)
                .build();

        Trajectory forwardOfSpikeMarks = drive.trajectoryBuilder(strafeRightClearOfSubmersible.end())
                .forward(30)
                .build();

        Trajectory strafeToSpikeThree = drive.trajectoryBuilder(forwardOfSpikeMarks.end())
                .strafeRight(13)
                .build();

        Trajectory strafeToSpikeTwo = drive.trajectoryBuilder(forwardOfSpikeMarks.end())
                .strafeRight(22)
                .build();

        Trajectory pushToObservationZoneSpikeThree = drive.trajectoryBuilder(strafeToSpikeThree.end())
                .back(48)
                .build();


        Trajectory pushToObservationZoneSpikeTwo = drive.trajectoryBuilder(strafeToSpikeTwo.end())
                .back(48)
                .build();

        Trajectory lineUpForSpecimenPickUp = drive.trajectoryBuilder(pushToObservationZoneSpikeThree.end())
                .lineToLinearHeading(new Pose2d(48, -43, Math.toRadians(270)))
                .build();

        Trajectory pickUpSpecimenOne = drive.trajectoryBuilder(lineUpForSpecimenPickUp.end())
                .forward(8)
                .build();

        Trajectory alignToScoreFromFirstPickUp = drive.trajectoryBuilder(pickUpSpecimenOne.end())
                .lineToLinearHeading(new Pose2d(6, -58, Math.toRadians(90)))
                .build();

        Trajectory parkInObservation = drive.trajectoryBuilder(pushToObservationZoneSpikeTwo.end())
                .lineToLinearHeading(new Pose2d(63, -60, Math.toRadians(90)))
                .build();
        waitForStart();
        drive.followTrajectory(moveOffWall);
        safelyRaiseArmForHang(robot);
        drive.followTrajectory(alignToScore);
        drive.followTrajectory(trajectoryScore);
        safelyDepositSpecimen(robot, drive, lockItIn);
        drive.followTrajectory(backUpFromHang);
        drive.followTrajectory(strafeRightClearOfSubmersible);
        drive.followTrajectory(forwardOfSpikeMarks);
        drive.followTrajectory(pushToObservationZoneSpikeThree);
        drive.followTrajectory(lineUpForSpecimenPickUp);
        robot.raiseForSpecimenCollect();
        drive.followTrajectory(pickUpSpecimenOne);
        robot.closeClaw();
        safelyRaiseArmForHang(robot);
        drive.followTrajectory(alignToScoreFromFirstPickUp);
        drive.followTrajectory(trajectoryScore);
        safelyDepositSpecimen(robot, drive, lockItIn);
        drive.followTrajectory(backUpFromHang);
        robot.storeRobot();

        robot.storeRobot();
        drive.followTrajectory(parkInObservation);


        telemetry.update();


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

    private void collectSampleWithClaw(Robot robot) {
        runtime.reset();
//        while (runtime.seconds() <= 1.5) {
//            robot.collectSample();
//        }
        runtime.reset();
        while (runtime.seconds() <= 1) {
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
