package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Core.vvHardwareITD;

/**
 * ITD (into the deep) teleOp for test and shakedown
 *
 * Need to confirm the arm, extender, wrist, and claw to have a full test case
 * Also need the telemetry to read all sensor values
 */

@TeleOp(name="vvTeleOp", group="1-TeleOp")

public class vvTeleOp extends LinearOpMode {

    //vvHardware class external pull
    vvHardwareITD   robot       = new vvHardwareITD(this);

    @Override
    public void runOpMode() throws InterruptedException {
        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        int x = 0;
        int y = 0;
        double drivePower = 0.5; //global drive power level
        double armPower = 0.5;
        double extPower = 0.5;
        int extLoc = 0;
        double wristPos = 0;
        double clawPos = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (isStopRequested()) return;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            while (opModeIsActive()) {
                driveY = -gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x * 1;
                turn = gamepad1.right_stick_x;
                //armPower = -gamepad2.left_stick_y;
                //extPower = -gamepad2.right_stick_y * 0.75;

                y = robot.parallelEncoder.getCurrentPosition();
                x = robot.perpendicularEncoder.getCurrentPosition(); //parallel, forward encoder distance is 0

                if (gamepad1.right_bumper) {
                    // button is transitioning to a pressed state. So increment drivePower by 0.1
                    drivePower = Math.min(drivePower + 0.05,0.9);
                }
                else if (gamepad1.left_bumper) {
                    // button is transitioning to a pressed state. So increment drivePower by -0.1
                    drivePower = Math.max(drivePower - 0.05, 0.1);
                }

                // Methods called for motion
                robot.driveRobot(drivePower, driveY, strafe, turn);

                //robot.moveArm(armPower);

                //robot.moveExt(extPower);

                wristPos = robot.wrist.getPosition();
                clawPos = robot.claw.getPosition();

                if (gamepad2.right_bumper)
                    robot.openClaw();
                if (gamepad2.left_bumper)
                    robot.closeClaw();
                if (gamepad2.x)
                    robot.longClaw();

                if (gamepad2.a) {
                    robot.armPos(robot.floorArm, armPower);
                    robot.extArmPos(robot.extArmFLoorPick,extPower);
                    robot.moveWristFloor();
                }
                if (gamepad2.b) {
                    robot.armPos(robot.floorArm, armPower);
                    robot.extArmPos(robot.extArmFLoorPick,extPower);
                    robot.moveWristCarry();


                }

                if (gamepad2.dpad_up) {
                    robot.armPos(robot.armHighBa, armPower);
                    robot.extArmPos(robot.extArmHighBe, extPower);
                    robot.moveWristHighBw();
                }
                if (gamepad2.dpad_right) {
                    robot.armPos(robot.armLowBa, armPower);
                    robot.extArmPos(robot.extArmLowBe, extPower);
                    robot.moveWristLowBw();
                }
                if (gamepad2.dpad_left) {
                    robot.armPos(robot.armHighCa, armPower);
                    robot.extArmPos(robot.armHighCa, extPower);
                    robot.moveWristHighCw();
                }
                if (gamepad2.dpad_down) {
                    robot.armPos(robot.armLowCa, armPower);
                    robot.extArmPos(robot.extArmLowCe, extPower);
                    robot.moveWristLowCW();
                }

                // Retrieve Rotational Angles and Velocities
                YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES);

// Adding telemetry readouts
                telemetry.addData(">", "Robot Running");
                telemetry.addData("Drive Power", drivePower);
                telemetry.addData("Y", driveY);
                telemetry.addData("strafe", strafe);
                telemetry.addData("turn", turn);
                telemetry.addData("Y Encoder",y);
                telemetry.addData("X Encoder",x);
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Arm Power", armPower);
                telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
                //telemetry.addData("Arm Target", robot.rightArm.getTargetPosition());
                telemetry.addData("Extend Position", robot.extend.getCurrentPosition());
                telemetry.addData("Wrist", wristPos);
                telemetry.addData("Claw", clawPos);


                telemetry.update();

                // Pace this loop so jaw action is reasonable speed.
                sleep(50);

            }
        }
    }
}

