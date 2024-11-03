package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.VfHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 */
@Config
@Autonomous(group = "2")
public class LowBasketDrop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        VfHardware robot = new VfHardware(this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(-27, -63, Math.toRadians(90));

        drive.setPoseEstimate(start);

        Trajectory trajectoryScore = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(225)))
                .build();

        Trajectory observationZone = drive.trajectoryBuilder(trajectoryScore.end())
                .lineToLinearHeading(new Pose2d(95, -63, Math.toRadians(90)))
                .build();

        waitForStart();

        drive.followTrajectory(trajectoryScore);
        robot.pause(1);
        robot.raiseForHighSpecimenHang();
        waitForIt();
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
