package org.firstinspires.ftc.teamcode.Auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
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

// Auton with 2 high chamber and park

    /*
     * Auton Blue Sequence
     * Start the robot on the x tile line against the wall
     *
     */
    @Autonomous(name = "vvHighCmbr", group = "2 - Auton", preselectTeleOp="vvTeleOp")

    public class vvHighCmbr extends LinearOpMode {
        vvHardwareITDRR robot = new vvHardwareITDRR(this);

        private ElapsedTime runtime = new ElapsedTime();

        @Override
        public void runOpMode() {

            vvRoadRunnerDrive vvdrive = new vvRoadRunnerDrive(hardwareMap);

            // We want to start the bot at x: 14, y: -60, heading: 90 degrees
            Pose2d startPose = new Pose2d(17, 65, Math.toRadians(270));

            vvdrive.setPoseEstimate(startPose);

            TrajectorySequence fwdHighChmbr = vvdrive.trajectorySequenceBuilder(startPose) //Also Red Back
                    .forward(25)
                    .waitSeconds(0.5)
                    .build();
            TrajectorySequence sample1Pick  = vvdrive.trajectorySequenceBuilder(fwdHighChmbr.end())
                    .back(6)
                    .turn(Math.toRadians(180))
                    .strafeLeft(83)
                    .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                        robot.armPos(robot.armWall, robot.armEPower);
                        robot.moveWristWall();
                        robot.extArmPos(50,robot.extArmEPower); })
                    .forward(10)
                    .build();
            TrajectorySequence sample1drop = vvdrive.trajectorySequenceBuilder(sample1Pick.end())
                    .strafeRight(50)
                    .turn(Math.toRadians(180))
                    .forward (12)
                    .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                        robot.armPos(robot.armHighCa, robot.armEPower);
                        robot.moveWristWall();
                        robot.extArmPos(robot.extArmHighCe,robot.extArmEPower );
                    })
                    .build();
            TrajectorySequence sample2Pick = vvdrive.trajectorySequenceBuilder(sample1drop.end()) //Also Blue Back
                    .turn(Math.toRadians(180))
                    .strafeLeft(53)
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                        robot.armPos(robot.armWall, robot.armEPower);
                        robot.moveWristWall();
                    })
                    .forward(10)
                    .build();
            TrajectorySequence sample2Drop = vvdrive.trajectorySequenceBuilder(sample2Pick.end())
                    .strafeRight(48)
                    .turn(Math.toRadians(180))
                    .forward (12  )
                    .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                        robot.armPos(robot.armHighCa, robot.armEPower);
                        robot.moveWristWall();
                        robot.extArmPos(robot.extArmHighCe,robot.extArmEPower );
                    })
                    .build();
            TrajectorySequence park = vvdrive.trajectorySequenceBuilder(sample2Drop.end())
                    .strafeLeft(48)
                    .back(24)
                    .build();
           /* TrajectorySequence yellowBackDropTopBlue = vvdrive.trajectorySequenceBuilder(purpleDropTopRed.end())
                    .back(3)
                    .turn(Math.toRadians(90))
                    .forward(44)
                    .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(armLow - 20, armEPower))
                    .strafeRight(8)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(autonPickupLow - 3, pickUpPwr))
                    .forward(2)
                    .build();
            TrajectorySequence yellowBackDropRightBlue = vvdrive.trajectorySequenceBuilder(purpleDropRightRed.end())
                    .back(6)
                    .turn(Math.toRadians(135))
                    .forward(42)
                    .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(armLow - 20, armEPower))
                    .strafeRight(9)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(autonPickupLow - 3, pickUpPwr))
                    .forward(2)
                    .build();
            TrajectorySequence yellowBackDropLeftBlue = vvdrive.trajectorySequenceBuilder(purpleDropLeftRed.end())
                    .strafeLeft(10)
                    .turn(Math.toRadians(35))
                    .forward(44)
                    .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(armLow - 20, armEPower))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(autonPickupLow - 3, pickUpPwr))
                    .forward(4)
                    .build();
            TrajectorySequence yellowBackDropTopRed = vvdrive.trajectorySequenceBuilder(purpleDropTopBlue.end())
                    .turn(Math.toRadians(-90))
                    .forward(48)
                    .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(armLow, armEPower))
                    .strafeRight(6)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(autonPickupLow, pickUpPwr))
                    .forward(4)
                    .build();
            TrajectorySequence yellowBackDropRightRed = vvdrive.trajectorySequenceBuilder(purpleDropRightBlue.end())
                    .turn(Math.toRadians(45))
                    .strafeLeft(6)
                    .forward(48)
                    .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(armLow, armEPower))
                    .strafeRight(8)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(autonPickupLow, pickUpPwr))
                    .forward(4)
                    .build();
            TrajectorySequence yellowBackDropLeftRed = vvdrive.trajectorySequenceBuilder(purpleDropLeftBlue.end())
                    .turn(Math.toRadians(135))
                    .forward(48)
                    .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(armLow, armEPower))
                    .strafeRight(6)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(autonPickupLow, pickUpPwr))
                    .forward(4)
                    .build();
            TrajectorySequence blueTopEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropTopBlue.end())
                    .back(6)
                    .strafeLeft(48)
                    .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(autonPickupIdle, pickUpPwr))
                    .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(armIdle, armEPower))
                    .build();
            TrajectorySequence blueRightEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropRightBlue.end())
                    .back(6)
                    .strafeLeft(56)
                    .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(autonPickupIdle, pickUpPwr))
                    .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(armIdle, armEPower))
                    .build();
            TrajectorySequence blueLeftEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropLeftBlue.end())
                    .back(6)
                    .strafeLeft(36)
                    .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(autonPickupIdle, pickUpPwr))
                    .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(armIdle, armEPower))
                    .build();
            TrajectorySequence redTopEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropTopRed.end())
                    .back(6)
                    .strafeRight(48)
                    .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(autonPickupIdle, pickUpPwr))
                    .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(armIdle, armEPower))
                    .build();
            TrajectorySequence redRightEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropRightRed.end())
                    .back(4)
                    .strafeRight(36)
                    .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(autonPickupIdle, pickUpPwr))
                    .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(armIdle, armEPower))
                    .build();
            TrajectorySequence redLeftEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropLeftRed.end())
                    .back(6)
                    .strafeRight(56)
                    .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(autonPickupIdle, pickUpPwr))
                    .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(armIdle, armEPower))
                    .build();
*/

            robot.init();

            // Wait for the DS start button to be touched.
            telemetry.addData(">", "Robot Ready");
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");

            waitForStart();

            if (opModeIsActive()) {
                while (opModeIsActive()) {

                    Pose2d poseEstimate = vvdrive.getPoseEstimate();
                    vvdrive.update();



                    telemetry.addData("Parallel Position: ", poseEstimate.getX());
                    telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                    telemetry.update();
                    robot.armPos(robot.armHighCa, robot.armEPower);
                    robot.moveWristHighCw();
                    robot.extArmPos(robot.extArmHighCe, robot.extArmEPower);
                    vvdrive.followTrajectorySequence(fwdHighChmbr);
                    sleep(500);
                    robot.armPos(robot.armHighCa-150,robot.armEPower );
                    sleep(500);
                    robot.openClaw();
                    vvdrive.followTrajectorySequence(sample1Pick);
                    sleep(500);
                    robot.closeClaw();
                    robot.armPos(robot.armWall+50,robot.armEPower );
                    vvdrive.followTrajectorySequence(sample1drop);
                    sleep(500);
                    robot.armPos(robot.armHighCa-100,robot.armEPower );
                    robot.openClaw();
                    sleep(500);
                    vvdrive.followTrajectorySequence(sample2Pick);
                    sleep(500);
                    robot.closeClaw();
                    robot.armPos(robot.armWall+50,robot.armEPower );
                    vvdrive.followTrajectorySequence(sample2Drop);
                    sleep(500);
                    robot.armPos(robot.armHighCa-100,robot.armEPower );
                    robot.openClaw();
                    telemetry.addData("Parallel Position: ", poseEstimate.getX());
                    telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                    telemetry.update();

                    break;
                }
            }
        }
    }

