package org.firstinspires.ftc.teamcode.Auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Core.vvHardware;
import org.firstinspires.ftc.teamcode.Core.vvHardwareITDRR;
import org.firstinspires.ftc.teamcode.Core.vvRoadRunnerDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

/*
 * High Basket Sequence
 * Start the robot on the furthest tile edge from the truss (left side)
 *
 */
@Autonomous(name = "vvHighBskt", group = "1 - Auton")

public class  vvHighBskt extends LinearOpMode {
    vvHardwareITDRR robot = new vvHardwareITDRR(this);

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        vvRoadRunnerDrive vvdrive = new vvRoadRunnerDrive(hardwareMap);

        // We want to start the bot at x: 14, y: -60, heading: 90 degrees
        Pose2d startPose = new Pose2d(17, 65, Math.toRadians(270));

        vvdrive.setPoseEstimate(startPose);

        TrajectorySequence fwdHighCmbr = vvdrive.trajectorySequenceBuilder(startPose) //Also Red Back
                .forward(24)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.armPos(robot.armHighCa, robot.armEPower))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.moveWristHighCw())
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmHighCe, robot.armEPower))
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow1 = vvdrive.trajectorySequenceBuilder(fwdHighCmbr.end())
                .back(2)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmFLoorPick, robot.armEPower))
                .turn(Math.toRadians(-90))
                .forward(24)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(robot.floorArm, robot.armEPower))
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow1Drop = vvdrive.trajectorySequenceBuilder(yellow1.end())
                .turn(Math.toRadians(-45))
                .forward(30)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(robot.armHighBa, robot.armEPower))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> robot.moveWristHighBw())
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmHighBe, robot.armEPower))
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow2 = vvdrive.trajectorySequenceBuilder(yellow1Drop.end()) //Also Blue Back
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmFLoorPick, robot.armEPower))
                .turn(Math.toRadians(45))
                .forward(30)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(robot.floorArm, robot.armEPower))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> robot.moveWristFloor())
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow2Drop = vvdrive.trajectorySequenceBuilder(yellow2.end())
                .back(5)
                .turn(Math.toRadians(-135))
                .forward(24)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> robot.armPos(robot.armHighBa, robot.armEPower))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> robot.moveWristHighBw())
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmHighBe, robot.armEPower))
                .waitSeconds(0)
                .build();
        TrajectorySequence ascentPark = vvdrive.trajectorySequenceBuilder(yellow2Drop.end())
                .back(48)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.extArmPos(robot.extArmFLoorPick, robot.armEPower))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(robot.floorArm, robot.armEPower))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.moveWristFloor())
                .waitSeconds(0)
                .build();

        robot.init();

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Robot Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                Pose2d poseEstimate = vvdrive.getPoseEstimate();
                vvdrive.update();

                vvdrive.followTrajectorySequence(fwdHighCmbr);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.armPos(robot.armHighCa,robot.armEPower);
                robot.openClaw();
                vvdrive.followTrajectorySequence(yellow1);
                robot.extArmPos(robot.extArmFLoorPick,robot.armEPower);
                sleep(1000);
                robot.closeClaw();
                vvdrive.followTrajectorySequence(yellow1Drop);
                robot.openClaw();
                vvdrive.followTrajectorySequence(yellow2);
                robot.closeClaw();
                vvdrive.followTrajectorySequence(yellow2Drop);
                robot.openClaw();
                vvdrive.followTrajectorySequence(ascentPark);
                robot.closeClaw();
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();

                break;
            }
        }
    }
}
