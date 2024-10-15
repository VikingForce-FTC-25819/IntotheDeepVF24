/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
 * Auton Blue - Backstage Side
 * Start the robot on the furthest tile edge from the truss (left side)
 *
 */
@Autonomous(name = "vvSnglBskt", group = "1 - Auton")

public class  vvSnglBskt extends LinearOpMode {
    vvHardwareITDRR robot = new vvHardwareITDRR(this);

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
    private static final String[] LABELS = {
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

        TrajectorySequence fwdHighCmbr = vvdrive.trajectorySequenceBuilder(startPose) //Also Red Back
                .forward(36)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.armPos(robot.armHighCa, armEPower))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.moveWristHighCw())
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmHighCe, armEPower))
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow = vvdrive.trajectorySequenceBuilder(fwdHighCmbr.end())
                .back(2)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmFLoorPick, armEPower))
                .turn(Math.toRadians(-90))
                .forward(24)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(robot.floorArm, armEPower))
                .waitSeconds(0)
                .build();
        TrajectorySequence yellowDrop = vvdrive.trajectorySequenceBuilder(yellow.end())
                .turn(Math.toRadians(-45))
                .forward(30)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(robot.armHighBa, armEPower))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> robot.moveWristHighBw())
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmHighBe, armEPower))
                .waitSeconds(0)
                .build();
        TrajectorySequence pickSpecimen = vvdrive.trajectorySequenceBuilder(yellowDrop.end()) //Also Blue Back
                .strafeRight(96)
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmFLoorPick, armEPower))
                .turn(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(0,-36),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.armPos(robot.armHighCa, armEPower))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.moveWristHighCw())
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmHighCe, armEPower))
                .waitSeconds(0)
                .build();
        TrajectorySequence observPark = vvdrive.trajectorySequenceBuilder(pickSpecimen.end())
                .splineToConstantHeading(new Vector2d(48,-65),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmFLoorPick, armEPower))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(robot.floorArm, armEPower))
                .waitSeconds(0)
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

                Pose2d poseEstimate = vvdrive.getPoseEstimate();
                vvdrive.update();

                vvdrive.followTrajectorySequence(fwdHighCmbr);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.armPos(robot.armHighCa,armEPower);
                robot.openClaw();
                vvdrive.followTrajectorySequence(yellow);
                robot.armPos(robot.extArmFLoorPick,armEPower);
                sleep(1000);
                robot.closeClaw();
                vvdrive.followTrajectorySequence(yellowDrop);
                robot.openClaw();
                vvdrive.followTrajectorySequence(pickSpecimen);
                robot.closeClaw();
                vvdrive.followTrajectorySequence(observPark);
                robot.openClaw();
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();

                break;
            }
        }
    }
}

// Save more CPU resources when camera is no longer needed.
//visionPortal.close();
// end runOpMode()

    /*
     * Initialize the TensorFlow Object Detection processor.

    protected String startDetection() {
        String teamPropPosition = "NOTFOUND";

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {

            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());


            if (recognition.getLabel() == "RedProp" || recognition.getLabel() == "BlueProp") {

                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                double propHeight = (recognition.getBottom() - recognition.getTop());
                double propWidth = (recognition.getRight() - recognition.getLeft());

                telemetry.addLine("propHeight:" + propHeight + ", propWidth:" + propWidth);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);

                if (propHeight > 70 && propHeight < 170 && propWidth > 70 && propWidth < 170) {
                    if (x < 200) {
                        teamPropPosition = "LEFT";
                    } else if (x >= 201 && x < 500) {
                        teamPropPosition = "CENTER";
                    } else {
                        teamPropPosition = "RIGHT";
                    }
                    break;
                }
            }
        }   // end for() loop
        telemetry.addLine("after for loop in getTeamPropPosition");
        visionPortal.stopStreaming();
        return teamPropPosition;
    }

    public void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    public void locTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method locTfod()
}*/
