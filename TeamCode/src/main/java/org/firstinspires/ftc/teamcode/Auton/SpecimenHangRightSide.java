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

        Trajectory shiftLeft = drive.trajectoryBuilder(moveOffWall.end())
                .strafeLeft(8)
                .build();

        Trajectory trajectoryScore = drive.trajectoryBuilder(shiftLeft.end())
                .forward(22)
                .build();

        Trajectory lockItIn = drive.trajectoryBuilder(trajectoryScore.end())
                .forward(4.5,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        Trajectory backUpFromHang = drive.trajectoryBuilder(lockItIn.end())
                .back(17)
                .build();


        Trajectory parkInObservation = drive.trajectoryBuilder(backUpFromHang.end())
                .lineToLinearHeading(new Pose2d(63, -60, Math.toRadians(90)))
                .build();


        waitForStart();
        drive.followTrajectory(moveOffWall);
        runtime.reset();
        while (runtime.seconds() <= .3) {
            robot.raiseForSpecimenHang();
        }
        drive.followTrajectory(shiftLeft);
        drive.followTrajectory(trajectoryScore);
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
        drive.followTrajectory(backUpFromHang);
        while (runtime.seconds() <= 3) {
            // be patient
        }
        robot.storeRobot();
        drive.followTrajectory(parkInObservation);


        telemetry.update();


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
