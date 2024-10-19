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
 * High Basket Sequence, two yellow picks to basket
 * Start the robot on the X tile line against the wall
 *
 */
@Autonomous(name = "vvHighBskt", group = "1 - Auton", preselectTeleOp="vvTeleOp")

public class  vvHighBskt extends LinearOpMode {
    vvHardwareITDRR robot = new vvHardwareITDRR(this);

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        vvRoadRunnerDrive vvdrive = new vvRoadRunnerDrive(hardwareMap);

        // We want to start the bot at x: 14, y: -60, heading: 90 degrees
        Pose2d startPose = new Pose2d(-12, -65, Math.toRadians(90));

        vvdrive.setPoseEstimate(startPose);

        TrajectorySequence fwdHighCmbr = vvdrive.trajectorySequenceBuilder(startPose) //Tile Start Position
                .forward(25)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.armPos(robot.armHighCa, robot.armEPower);
                    robot.moveWristHighCw();
                    robot.extArmPos(robot.extArmHighCe, robot.armEPower);
                })
                .waitSeconds(0.5)
                .build();
        TrajectorySequence yellow1 = vvdrive.trajectorySequenceBuilder(fwdHighCmbr.end())
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> robot.extArmPos(0, robot.armEPower))
                .strafeLeft(56)
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristFloor();
                    robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower);
                })
                .waitSeconds(0.5)
                .build();
        TrajectorySequence yellow1Drop = vvdrive.trajectorySequenceBuilder(yellow1.end())
                .turn(Math.toRadians(135))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.armHighBa, robot.armEPower);
                    robot.moveWristHighBw();
                })
                .forward(22)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.extArmPos(robot.extArmHighBe, robot.extArmEPower))
                .waitSeconds(1)
                .build();
        TrajectorySequence yellow2 = vvdrive.trajectorySequenceBuilder(yellow1Drop.end()) //Also Blue Back
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmFLoorPick, robot.armEPower))
                .turn(Math.toRadians(-135))
                .forward(24)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristFloor();
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence yellow2Drop = vvdrive.trajectorySequenceBuilder(yellow2.end())
                .back(5)
                .turn(Math.toRadians(-180))
                .forward(24)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.armHighBa, robot.armEPower);
                    robot.moveWristHighBw();
                })
                .turn(Math.toRadians(-45))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmHighBe, robot.armEPower))
                .waitSeconds(1)
                .build();
        TrajectorySequence ascentPark = vvdrive.trajectorySequenceBuilder(yellow2Drop.end())
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(0, robot.armEPower))
                .turn(Math.toRadians(45))
                .back(48)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(0, robot.armEPower);
                    robot.moveWristCarry();
                })
                .strafeLeft(12)
                .waitSeconds(1)
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
                sleep(500);
                robot.armPos(robot.armHighCa-100,robot.armEPower);
                sleep(500);
                robot.openClaw();
                sleep(500);
                vvdrive.followTrajectorySequence(yellow1);
                sleep(500);
                robot.closeClaw();
                sleep(250);
                vvdrive.followTrajectorySequence(yellow1Drop);
                robot.openClaw();
                sleep(500);
                vvdrive.followTrajectorySequence(yellow2);
                robot.closeClaw();
                sleep(500);
                vvdrive.followTrajectorySequence(yellow2Drop);
                robot.openClaw();
                sleep(500);
                vvdrive.followTrajectorySequence(ascentPark);
                robot.closeClaw();
                sleep(500);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();

                break;
            }
        }
    }
}
