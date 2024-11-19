package org.firstinspires.ftc.teamcode.Auton;

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
@Autonomous(group = "2")
public class RightStartHighBasket extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(16, -63, Math.toRadians(90));

        drive.setPoseEstimate(start);

        Trajectory moveOffWall = drive.trajectoryBuilder(start)
                .forward(3)
                .build();

        Trajectory trajectoryScore = drive.trajectoryBuilder(moveOffWall.end())
                .lineToLinearHeading(new Pose2d(-59, -54, Math.toRadians(225)))
                .build();

        Trajectory observationZone = drive.trajectoryBuilder(trajectoryScore.end())
                .lineToLinearHeading(new Pose2d(60, -60, Math.toRadians(90)))
                .build();

        waitForStart();
        runtime.reset();
        while (runtime.seconds() <= 3) {
            // do nothing for 3 seconds
        }
        drive.followTrajectory(moveOffWall);
        for (int i = 0; i < 2; i++) {
            robot.raiseForHighBasket();
        }
        drive.followTrajectory(trajectoryScore);
        runtime.reset();
        while (runtime.seconds() <= 2.5) {
            robot.deposit();
        }
        robot.storeRobot();
        drive.followTrajectory(observationZone);

        telemetry.update();


    }

    private static void waitForIt() {
        long startTime = System.currentTimeMillis();
        long duration = 3000; // 5 seconds in milliseconds

        while (System.currentTimeMillis() - startTime < duration) {
            //do nothing to let the deposit keep going....
        }
    }
}
