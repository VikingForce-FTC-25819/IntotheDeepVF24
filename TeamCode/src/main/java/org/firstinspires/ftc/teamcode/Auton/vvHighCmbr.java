package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Core.vvHardware;
import org.firstinspires.ftc.teamcode.Core.vvHardwareRR;
import org.firstinspires.ftc.teamcode.Core.vvRoadRunnerDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

// Auton with 2 high chamber and parK

    /*
     * Auton Blue - Backstage Side
     * Start the robot on the furthest tile edge from the truss (left side)
     *
     */
    @Autonomous(name = "vvHighCmbr", group = "1 - Blue Auton")

    public class vvHighCmbr extends LinearOpMode {
        vvHardwareRR robot = new vvHardwareRR(this);

        private ElapsedTime runtime = new ElapsedTime();

        static final double FORWARD_SPEED = 0.3;
        static final double TURN_SPEED = 0.5;
        // the amount of time the pickup takes to activate in seconds
        final double pickupTime = 1;
        // the amount of time the arm takes to raise in seconds
        final double armTime = 2;
        final int armIdle = 0;
        final int armLow = 120; // the low encoder position for the arm, front place
        final int armHigh = 401; // the high-overhead encoder position for the arm
        final int armStart = 25;
        double armEPower = 0.8;
        double pickUpPwr = 0.7;
        final int autonPickupIdle = -30; // the idle position for the pickup motor 109
        final int autonPickupHigh = -5; // the placing position for the pickup motor in the high position 148
        final int autonPickupLow = -27; // the placing position for the pickup motor in the low/forward position 5

        private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

        // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
        // this is only used for Android Studio when using models in Assets.
        private static final String TFOD_MODEL_ASSET = "vvCenterStage7Nov.tflite";
        // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
        // this is used when uploading models directly to the RC using the model upload interface.
        //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/vvCenterStage7Nov.tflite";
        // Define the labels recognized in the model for TFOD (must be in training order!)
        private final String[] LABELS = {
                "BlueProp", "Pixel", "RedProp"
        };

        /**
         * The variable to store our instance of the TensorFlow Object Detection processor.
         */
        //private TfodProcessor tfod;

        /**
         * The variable to store our instance of the vision portal.
         */
        private VisionPortal visionPortal;

        @Override
        public void runOpMode() {

            vvRoadRunnerDrive vvdrive = new vvRoadRunnerDrive(hardwareMap);

            // We want to start the bot at x: 14, y: -60, heading: 90 degrees
            Pose2d startPose = new Pose2d(17, 65, Math.toRadians(270));

            vvdrive.setPoseEstimate(startPose);

            TrajectorySequence purpleDropTopBlue = vvdrive.trajectorySequenceBuilder(startPose) //Also Red Back
                    .forward(35)
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(autonPickupLow, pickUpPwr))
                    .back(10)
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(armIdle + 5, armEPower))
                    .waitSeconds(1)
                    .build();
            TrajectorySequence purpleDropLeftBlue = vvdrive.trajectorySequenceBuilder(startPose)
                    .forward(30)
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(autonPickupLow, pickUpPwr))
                    .turn(Math.toRadians(60))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.armPos(armIdle + 5, armEPower))
                    .forward(9)
                    .waitSeconds(1)
                    .back(3)
                    .build();
            TrajectorySequence purpleDropRightBlue = vvdrive.trajectorySequenceBuilder(startPose)
                    .forward(38)
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(autonPickupLow, pickUpPwr))
                    .turn(Math.toRadians(-45))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.armPos(armIdle + 5, armEPower))
                    .back(4)
                    .waitSeconds(1)
                    .build();
            TrajectorySequence purpleDropTopRed = vvdrive.trajectorySequenceBuilder(startPose) //Also Blue Back
                    .forward(33)
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.movePickUp(autonPickupIdle - 5, pickUpPwr))
                    .back(8)
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(armIdle + 5, armEPower))
                    .waitSeconds(1)
                    .build();
            TrajectorySequence purpleDropLeftRed = vvdrive.trajectorySequenceBuilder(startPose)
                    .forward(27)
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(autonPickupLow, pickUpPwr))
                    .turn(Math.toRadians(60))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.armPos(armIdle + 5, armEPower))
                    .forward(9)
                    .waitSeconds(1)
                    .back(9)
                    .build();
            TrajectorySequence purpleDropRightRed = vvdrive.trajectorySequenceBuilder(startPose)
                    .forward(22)
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(autonPickupLow, pickUpPwr))
                    .turn(Math.toRadians(-50))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.armPos(armIdle + 5, armEPower))
                    .forward(6)
                    .waitSeconds(1)
                    .build();
            TrajectorySequence yellowBackDropTopBlue = vvdrive.trajectorySequenceBuilder(purpleDropTopRed.end())
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

            robot.init();
            //initTfod();
            String spikeLoc;

            // Wait for the DS start button to be touched.
            telemetry.addData(">", "Robot Ready");
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();

            waitForStart();

            if (opModeIsActive()) {
                while (opModeIsActive()) {

                    //telemetryTfod();
                    spikeLoc = "Blue";
                    // Push telemetry to the Driver Station.
                    //telemetry.addData("Location Heading", spikeLoc);
                    telemetry.update();

                    // Share the CPU.
                    sleep(200);

                    Pose2d poseEstimate = vvdrive.getPoseEstimate();
                    vvdrive.update();

                    if (Objects.equals(spikeLoc, "LEFT")) {
                        robot.movePickUp(5, pickUpPwr);
                        sleep(500);
                        robot.armPos(armStart, armEPower);
                        sleep(500);
                        vvdrive.followTrajectorySequence(purpleDropLeftRed);
                        telemetry.addData("Parallel Position: ", poseEstimate.getX());
                        telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                        telemetry.update();
                        robot.rightWheel.setPower(-0.9);
                        sleep(1000);
                        robot.rightWheel.setPower(0);
                        robot.movePickUp(autonPickupLow, pickUpPwr);
                        sleep(500);
                        vvdrive.followTrajectorySequence(yellowBackDropLeftBlue);
                        robot.leftWheel.setPower(0.9);
                        sleep(1000);
                        robot.leftWheel.setPower(0);
                        vvdrive.followTrajectorySequence(blueLeftEnd);
                        telemetry.addData("Parallel Position: ", poseEstimate.getX());
                        telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                        telemetry.update();
                    }
                    if (Objects.equals(spikeLoc, "CENTER")) {
                        robot.movePickUp(5, pickUpPwr);
                        sleep(500);
                        robot.armPos(armStart, armEPower);
                        sleep(500);
                        vvdrive.followTrajectorySequence(purpleDropTopRed);
                        robot.movePickUp(autonPickupLow, pickUpPwr);
                        telemetry.addData("Parallel Position: ", poseEstimate.getX());
                        telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                        telemetry.update();
                        robot.rightWheel.setPower(-0.9);
                        sleep(1000);
                        robot.rightWheel.setPower(0);
                        vvdrive.followTrajectorySequence(yellowBackDropTopBlue);
                        telemetry.addData("Parallel Position: ", poseEstimate.getX());
                        telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                        telemetry.update();
                        robot.leftWheel.setPower(0.9);
                        sleep(1000);
                        robot.leftWheel.setPower(0);
                        vvdrive.followTrajectorySequence(blueTopEnd);
                        telemetry.addData("Parallel Position: ", poseEstimate.getX());
                        telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                        telemetry.update();
                    }
                    if (Objects.equals(spikeLoc, "RIGHT")) {
                        robot.movePickUp(5, pickUpPwr);
                        sleep(500);
                        robot.armPos(armStart, armEPower);
                        sleep(500);
                        vvdrive.followTrajectorySequence(purpleDropRightRed);
                        telemetry.addData("Parallel Position: ", poseEstimate.getX());
                        telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                        telemetry.update();
                        robot.rightWheel.setPower(-0.9);
                        sleep(1000);
                        robot.rightWheel.setPower(0);
                        vvdrive.followTrajectorySequence(yellowBackDropRightBlue);
                        robot.leftWheel.setPower(0.9);
                        sleep(1000);
                        robot.leftWheel.setPower(0);
                        vvdrive.followTrajectorySequence(blueRightEnd);
                        telemetry.addData("Parallel Position: ", poseEstimate.getX());
                        telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                        telemetry.update();
                    }
                    if (Objects.equals(spikeLoc, "NOTFOUND")) {//Right is the default
                        robot.movePickUp(5, pickUpPwr);
                        sleep(500);
                        robot.armPos(armStart, armEPower);
                        sleep(500);
                        vvdrive.followTrajectorySequence(purpleDropLeftRed);
                        telemetry.addData("Parallel Position: ", poseEstimate.getX());
                        telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                        telemetry.update();
                        robot.rightWheel.setPower(-0.9);
                        sleep(1000);
                        robot.rightWheel.setPower(0);
                        robot.movePickUp(autonPickupLow, pickUpPwr);
                        sleep(500);
                        vvdrive.followTrajectorySequence(yellowBackDropLeftBlue);
                        robot.leftWheel.setPower(0.9);
                        sleep(1000);
                        robot.leftWheel.setPower(0);
                        vvdrive.followTrajectorySequence(blueLeftEnd);
                        telemetry.addData("Parallel Position: ", poseEstimate.getX());
                        telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                        telemetry.update();
                    }
                    break;
                }
            }
        }
    }

