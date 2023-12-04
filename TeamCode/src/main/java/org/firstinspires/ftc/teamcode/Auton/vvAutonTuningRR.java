
package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Core.vvHardware;
import org.firstinspires.ftc.teamcode.Core.vvHardwareRR;
import org.firstinspires.ftc.teamcode.Core.vvRoadRunnerDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * Mr. Price's teleOp testing for use of switches in teleop
 *
 * Need to confirm the drive system and add the drone servo to have a full test case
 * Also need the telemetry to read all sensor values
 */

@TeleOp(name="vvTuningRR", group="2-TeleOp")

public class vvAutonTuningRR extends LinearOpMode {

    //vvHardware class external pull
    vvHardwareRR robot = new vvHardwareRR(this);

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();

        robot.init();

        vvRoadRunnerDrive vvdrive = new vvRoadRunnerDrive(hardwareMap);

        /** Starting locations on Centerstage field
         * Red Stage x: -41, y: -65, heading: 90 degrees
         * Blue Stage x: -41, y: 65, heading: 270 degrees
         * Red Back x: 17, y: -65, heading: 90 degrees
         * Blue Back x: 17, y: 65, heading: 270 degrees
        **/
        Pose2d startPose = new Pose2d(-41, -65, Math.toRadians(90));

        vvdrive.setPoseEstimate(startPose);

        TrajectorySequence purpleDropTopBlue = vvdrive.trajectorySequenceBuilder(startPose) //Also Red Back
                .forward(38)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(robot.armIdle + 5, robot.armEPower))
                .waitSeconds(1)
                .build();
        TrajectorySequence purpleDropLeftBlue = vvdrive.trajectorySequenceBuilder(startPose)
                .forward(28)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .turn(Math.toRadians(60))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.armPos(robot.armIdle + 5, robot.armEPower))
                .forward(9)
                .waitSeconds(1)
                .back(3)
                .build();
        TrajectorySequence purpleDropRightBlue = vvdrive.trajectorySequenceBuilder(startPose)
                .forward(34)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .turn(Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.armPos(robot.armIdle + 5, robot.armEPower))
                .back(4)
                .waitSeconds(1)
                .build();
        TrajectorySequence purpleDropTopRed = vvdrive.trajectorySequenceBuilder(startPose) //Also Blue Back
                .forward(38)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.movePickUp(robot.autonPickupIdle - 5, robot.pickUpPwr))
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(robot.armIdle + 5, robot.armEPower))
                .waitSeconds(1)
                .build();
        TrajectorySequence purpleDropLeftRed = vvdrive.trajectorySequenceBuilder(startPose)
                .forward(28)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .turn(Math.toRadians(60))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.armPos(robot.armIdle + 5, robot.armEPower))
                .forward(9)
                .waitSeconds(1)
                .back(9)
                .build();
        TrajectorySequence purpleDropRightRed = vvdrive.trajectorySequenceBuilder(startPose)
                .forward(32)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .turn(Math.toRadians(-50))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.armPos(robot.armIdle + 5, robot.armEPower))
                .forward(6)
                .waitSeconds(1)
                .build();
        TrajectorySequence yellowStageDropTopRed = vvdrive.trajectorySequenceBuilder(purpleDropTopRed.end())
                .turn(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-64, -14),Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(-12, () -> robot.armPos(robot.armStart+15, robot.armEPower))
                .UNSTABLE_addDisplacementMarkerOffset(-6, () -> robot.movePickUp(robot.autonPickupLow-15, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-1,() -> robot.rightWheel.setPower(-0.9))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> robot.rightWheel.setPower(0))
                .lineToConstantHeading(new Vector2d(-24, -7))
                .back(48)
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(robot.armLow+25, robot.armEPower))
                .turn(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(48,-36),Math.toRadians(0))
                .forward(6)
                .build();
        TrajectorySequence yellowBackDropTopBlue = vvdrive.trajectorySequenceBuilder(purpleDropTopRed.end())
                .back(3)
                .turn(Math.toRadians(90))
                .forward(44)
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(robot.armLow, robot.armEPower))
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .forward(2)
                .build();
        TrajectorySequence yellowBackDropRightBlue = vvdrive.trajectorySequenceBuilder(purpleDropRightRed.end())
                .back(6)
                .turn(Math.toRadians(135))
                .forward(42)
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(robot.armLow, robot.armEPower))
                .strafeRight(9)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .forward(4)
                .build();
        TrajectorySequence yellowBackDropLeftBlue = vvdrive.trajectorySequenceBuilder(purpleDropLeftRed.end())
                .strafeLeft(10)
                .turn(Math.toRadians(35))
                .forward(44)
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(robot.armLow, robot.armEPower))
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .forward(4)
                .build();
        TrajectorySequence yellowBackDropTopRed = vvdrive.trajectorySequenceBuilder(purpleDropTopBlue.end())
                .turn(Math.toRadians(-90))
                .forward(48)
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(robot.armLow, robot.armEPower))
                .strafeRight(4)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .forward(4)
                .build();
        TrajectorySequence yellowBackDropRightRed = vvdrive.trajectorySequenceBuilder(purpleDropRightBlue.end())
                .back(6)
                .turn(Math.toRadians(-45))
                .strafeRight(8)
                .forward(48)
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(robot.armLow, robot.armEPower))
                .strafeRight(11)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .forward(3)
                .build();
        TrajectorySequence yellowBackDropLeftRed = vvdrive.trajectorySequenceBuilder(purpleDropLeftBlue.end())
                .back(6)
                .turn(Math.toRadians(-145))
                .forward(48)
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(robot.armLow, robot.armEPower))
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .forward(4)
                .build();
        TrajectorySequence blueTopEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropTopBlue.end())
                .back(6)
                .strafeLeft(48)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        TrajectorySequence blueRightEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropRightBlue.end())
                .back(6)
                .strafeLeft(56)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        TrajectorySequence blueLeftEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropLeftBlue.end())
                .back(6)
                .strafeLeft(36)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        TrajectorySequence redTopEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropTopRed.end())
                .back(6)
                .strafeRight(48)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        TrajectorySequence redRightEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropRightRed.end())
                .back(4)
                .strafeRight(36)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        TrajectorySequence redLeftEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropLeftRed.end())
                .back(6)
                .strafeRight(56)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        TrajectorySequence redStageTopEnd = vvdrive.trajectorySequenceBuilder(yellowStageDropTopRed.end())
                .back(6)
                .strafeLeft(24)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        // initialize all the hardware
        robot.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (isStopRequested()) return;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            Pose2d poseEstimate = vvdrive.getPoseEstimate();
            vvdrive.update();

            if (gamepad1.dpad_up) { //Stage Top, Blue
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropTopBlue);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
            }
            if (gamepad1.dpad_left) { //Stage Left
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropLeftBlue);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
            }
            if (gamepad1.dpad_right) { //Stage Right
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropRightBlue);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
            }
            if (gamepad1.y) { //Backdrop Top Blue
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropTopRed);
                robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr);
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

            if (gamepad1.x){ //Backdrop Left Blue
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropLeftRed);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
                robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr);
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
            if(gamepad1.b){ //Backdrop Right Blue
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
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
            if (gamepad2.dpad_up) { //Stage Top, Red
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropTopRed);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
            }
            if (gamepad2.dpad_left) { //Stage Left
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropLeftRed);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
            }
            if (gamepad2.dpad_right) { //Stage Right
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropRightRed);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
            }
            if (gamepad2.dpad_down) { //Stage Top, Red to Backdrop
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropTopRed);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
                robot.armPos(robot.armStart, robot.armEPower);
                vvdrive.followTrajectorySequence(yellowStageDropTopRed);
                robot.leftWheel.setPower(0.9);
                sleep(1000);
                robot.leftWheel.setPower(0);
                vvdrive.followTrajectorySequence(redStageTopEnd);
            }
            if (gamepad2.y) { //Backdrop Top Red
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropTopBlue);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
                vvdrive.followTrajectorySequence(yellowBackDropTopRed);
                robot.leftWheel.setPower(0.9);
                sleep(1000);
                robot.leftWheel.setPower(0);
                vvdrive.followTrajectorySequence(redTopEnd);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
            }
            if (gamepad2.x){ //Backdrop Left Red
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropLeftBlue);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
                vvdrive.followTrajectorySequence(yellowBackDropLeftRed);
                robot.leftWheel.setPower(0.9);
                sleep(1000);
                robot.leftWheel.setPower(0);
                vvdrive.followTrajectorySequence(redLeftEnd);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
            }
            if(gamepad2.b){ //Backdrop Right Red
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropRightBlue);
                robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
                vvdrive.followTrajectorySequence(yellowBackDropRightRed);
                robot.leftWheel.setPower(0.9);
                sleep(1000);
                robot.leftWheel.setPower(0);
                vvdrive.followTrajectorySequence(redRightEnd);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
            }
            if(gamepad1.left_bumper) { //Pickup oscillation testing
                robot.armPos(robot.armLow, robot.armEPower);
                sleep(1000);
                robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr);
            };
        }
    }
}

