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

    public static final int SPECIMEN_THREE_X_AXIS = 11;
    public static final int SPECIMEN_TWO_X_AXIS = 7;
    public static final int SPECIMEN_ONE_X_AXIS = 4;
    public static final double SPECIMEN_COLLECT_Y_AXIS = -54.5;
    public static final int SPECIMEN_COLLECT_X_AXIS = 48;
    public static final int LOCK_IN_MAX_VEL = 35;
    private static final int LOCK_IN_MAX_ACC = 35;
    private static final double LOCK_IN_DISTANCE = 0.3;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(12.5, -63, Math.toRadians(90));

        drive.setPoseEstimate(start);

        Trajectory trajectoryScore = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(SPECIMEN_ONE_X_AXIS, -36, Math.toRadians(90)))
                .build();

        Trajectory lockItIn = drive.trajectoryBuilder(trajectoryScore.end())
                .forward(LOCK_IN_DISTANCE,
                        SampleMecanumDrive.getVelocityConstraint(LOCK_IN_MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(LOCK_IN_MAX_ACC))
                .build();

        Trajectory backUpFromHang = drive.trajectoryBuilder(lockItIn.end())
                .lineToLinearHeading(new Pose2d(SPECIMEN_ONE_X_AXIS, -42, Math.toRadians(90)))
                .build();

        Trajectory splineRightOfSubmersible = drive.trajectoryBuilder(backUpFromHang.end())
                .splineToLinearHeading(new Pose2d(36, -34, Math.toRadians(270)), Math.toRadians(115))
                .build();

        Trajectory forwardOfSpikeMarks = drive.trajectoryBuilder(splineRightOfSubmersible.end())
                .lineToLinearHeading(new Pose2d(38, -13, Math.toRadians(270)))
                .build();

        Trajectory strafeToSpikeThree = drive.trajectoryBuilder(forwardOfSpikeMarks.end())
                .lineToLinearHeading(new Pose2d(46, -13, Math.toRadians(270)))
                .build();


        Trajectory pushToObservationZoneSpikeThree = drive.trajectoryBuilder(strafeToSpikeThree.end())
                .lineToLinearHeading(new Pose2d(SPECIMEN_COLLECT_X_AXIS, SPECIMEN_COLLECT_Y_AXIS, Math.toRadians(270)),
                    getVelocityConstraint(LOCK_IN_MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                    getAccelerationConstraint(LOCK_IN_MAX_ACC))
                .build();

        Trajectory lineUpForSpecimenPickUpOne = drive.trajectoryBuilder(pushToObservationZoneSpikeThree.end())
                .lineToLinearHeading(new Pose2d(SPECIMEN_COLLECT_X_AXIS, SPECIMEN_COLLECT_Y_AXIS + 0.5, Math.toRadians(270)))
                .build();

        Trajectory pickUpSpecimenOne = drive.trajectoryBuilder(lineUpForSpecimenPickUpOne.end())
                .lineToLinearHeading(new Pose2d(SPECIMEN_COLLECT_X_AXIS, SPECIMEN_COLLECT_Y_AXIS, Math.toRadians(270)))
                .build();

        Trajectory alignToScoreFromFirstPickUp = drive.trajectoryBuilder(pickUpSpecimenOne.end())
                .lineToLinearHeading(new Pose2d(SPECIMEN_TWO_X_AXIS, -42, Math.toRadians(90)))
                .build();

        Trajectory trajectoryScoreFromFirstPickUp = drive.trajectoryBuilder(alignToScoreFromFirstPickUp.end())
                .lineToLinearHeading(new Pose2d(SPECIMEN_TWO_X_AXIS, -36, Math.toRadians(90)))
                .build();

        Trajectory lockItInSecondSpecimen = drive.trajectoryBuilder(trajectoryScoreFromFirstPickUp.end())
                .forward(LOCK_IN_DISTANCE,
                        SampleMecanumDrive.getVelocityConstraint(LOCK_IN_MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(LOCK_IN_MAX_ACC))
                .build();

        Trajectory backUpFromHangSecondSpecimen = drive.trajectoryBuilder(lockItInSecondSpecimen.end())
                .lineToLinearHeading(new Pose2d(SPECIMEN_TWO_X_AXIS, -42, Math.toRadians(90)))
                .build();

        Trajectory lineUpForSpecimenPickUpTwo = drive.trajectoryBuilder(backUpFromHangSecondSpecimen.end())
                .lineToLinearHeading(new Pose2d(SPECIMEN_COLLECT_X_AXIS, -52, Math.toRadians(270)))
                .build();

        Trajectory pickUpSpecimenTwo = drive.trajectoryBuilder(lineUpForSpecimenPickUpTwo.end())
                .lineToLinearHeading(new Pose2d(SPECIMEN_COLLECT_X_AXIS, SPECIMEN_COLLECT_Y_AXIS, Math.toRadians(270)))
                .build();

        Trajectory alignToScoreFromSecondPickUp = drive.trajectoryBuilder(pickUpSpecimenTwo.end())
                .lineToLinearHeading(new Pose2d(SPECIMEN_THREE_X_AXIS, -42, Math.toRadians(90)))
                .build();

        Trajectory trajectoryScoreFromSecondPickUp = drive.trajectoryBuilder(alignToScoreFromSecondPickUp.end())
                .lineToLinearHeading(new Pose2d(SPECIMEN_THREE_X_AXIS, -36, Math.toRadians(90)))
                .build();

        Trajectory lockItInThirdSpecimen = drive.trajectoryBuilder(trajectoryScoreFromSecondPickUp.end())
                .forward(LOCK_IN_DISTANCE,
                        SampleMecanumDrive.getVelocityConstraint(LOCK_IN_MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(LOCK_IN_MAX_ACC))
                .build();

        Trajectory backUpFromHangThirdSpecimen = drive.trajectoryBuilder(lockItInThirdSpecimen.end())
                .lineToLinearHeading(new Pose2d(SPECIMEN_THREE_X_AXIS, -42, Math.toRadians(90)))
                .build();

        Trajectory parkInObservation = drive.trajectoryBuilder(backUpFromHangThirdSpecimen.end())
                .lineToLinearHeading(new Pose2d(63, -60, Math.toRadians(90)))
                .build();
        waitForStart();
        //drive.followTrajectory(moveOffWall);
        safelyRaiseArmForHang(robot);
        telemetry.update();
        //drive.followTrajectory(alignToScore);
        drive.followTrajectory(trajectoryScore);
        safelyDepositSpecimen(robot, drive, lockItIn);
        telemetry.update();
        drive.followTrajectory(backUpFromHang);
        drive.followTrajectory(splineRightOfSubmersible);
        robot.raiseForSpecimenCollect();
        drive.followTrajectory(forwardOfSpikeMarks);
        drive.followTrajectory(strafeToSpikeThree);
        drive.followTrajectory(pushToObservationZoneSpikeThree);
        drive.followTrajectory(lineUpForSpecimenPickUpOne);
        telemetry.update();
        safelyApproachSpecimen(drive, pickUpSpecimenOne);
        collectSpecimenWithClaw(robot);
        telemetry.update();
        drive.followTrajectory(alignToScoreFromFirstPickUp);
        drive.followTrajectory(trajectoryScoreFromFirstPickUp);
        safelyDepositSpecimen(robot, drive, lockItInSecondSpecimen);
        drive.followTrajectory(backUpFromHangSecondSpecimen);
        // get that third specimen
        drive.followTrajectory(lineUpForSpecimenPickUpTwo);
        robot.raiseForSpecimenCollect();
        telemetry.update();
        safelyApproachSpecimen(drive, pickUpSpecimenTwo);
        collectSpecimenWithClaw(robot);
        telemetry.update();
        drive.followTrajectory(alignToScoreFromSecondPickUp);
        drive.followTrajectory(trajectoryScoreFromSecondPickUp);
        safelyDepositSpecimen(robot, drive, lockItInThirdSpecimen);
        drive.followTrajectory(backUpFromHangThirdSpecimen);

        // shut down and park
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
        while (runtime.seconds() <= 0.5) {
            // wait for human player
        }
        drive.followTrajectory(trajectory);
    }

    private void safelyRaiseArmForHang(Robot robot) {
        runtime.reset();
        while (runtime.seconds() <= .2) {
            robot.raiseForSpecimenHang();
        }
    }

    private void safelyDepositSpecimen(Robot robot, SampleMecanumDrive drive, Trajectory lockItIn) {
        robot.adjustArmAngleJerking(-0.5);
        runtime.reset();
        while (runtime.seconds() <= 0.2) {
            // be patient
        }
        drive.followTrajectory(lockItIn);
        runtime.reset();
        while (runtime.seconds() <= .2) {
            robot.deposit();
        }
    }

    private void collectSpecimenWithClaw(Robot robot) {
        runtime.reset();
        while (runtime.seconds() <= 0.5) {
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
