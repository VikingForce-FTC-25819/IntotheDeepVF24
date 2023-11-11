
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Core.vvHardware;

/**
 * Mr. Price's teleOp testing for use of switches in teleop
 *
 * Need to confirm the drive system and add the drone servo to have a full test case
 * Also need the telemetry to read all sensor values
 */

@TeleOp(name="swTeleOp", group="Concept")

public class teleOpSwitched extends LinearOpMode {

    //vvHardware class external pull
    vvHardware   robot       = new vvHardware(this);

    // An Enum is used to represent pickup state
    // (This is one thing enums are designed to do)
    public enum PickupState {
        PickupStart,
        PickupPick,
        PickupPlaceLow,
        PickupPlaceHigh
    };
    public enum PickupWheels {
        PickupLin,
        PickupLout,
        PickupRin,
        PickRout,
        PickupIdle
    }
    public enum ArmPosition {
        ArmIdle,
        ArmLow,
        ArmHigh,
        ArmLift
    }
    public ColorSensor colorSensor;
    public DistanceSensor distFront;
    public DistanceSensor distRear;

@Override
    public void runOpMode() throws InterruptedException {
        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        int x = 0;
        int y = 0;
        double RWPower = 0;
        double LWPower = 0;
        double RWPowerPU = 0;
        double LWPowerPU = 0;
        double drivePower = 0.5; //global drive power level
        double armPower = 0;
        double liftPower = 0;
        int liftLoc = 0;
        double armEPower = 0.9;
        double pickUpPwr = 0.9;
        double droneSet = 0.25;
        double droneLaunch = 0;
        double servpos = 0;

        // The pickupState variable is declared out here
        // so its value persists between loop() calls
        PickupState pickupState = PickupState.PickupStart;
        PickupWheels pickupWheels = PickupWheels.PickupIdle;
        ArmPosition armPosition = ArmPosition.ArmIdle;
        // used with the arm timing
        ElapsedTime pickupTimer = new ElapsedTime();
        ElapsedTime armTimer = new ElapsedTime();

        final int pickupIdle = -10; // the idle position for the pickup motor
        final int pickupPick = -28; // the pickup position
        final int pickupHigh= 0; // the placing position for the pickup motor in the high position
        final int pickupLow = -45; // the placing position for the pickup motor in the low/forward position

        // the amount of time the pickup takes to activate in seconds
        final double pickupTime;
        // the amount of time the arm takes to raise in seconds
        final double armTime = 2;
        final int armIdle = 0;
        final int armLow = 20; // the low encoder position for the arm, front place
        final int armHigh = 160; // the high-overhead encoder position for the arm
        final int armLift = 200; //lift location for arm

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        //pickupTimer.reset();

        // get a reference to our ColorSensor object.
        //colorSensor = hardwareMap.get(ColorSensor.class, "CLR");

        //distFront = hardwareMap.get(DistanceSensor.class, "FDS");
        //distRear = hardwareMap.get(DistanceSensor.class, "RDS");

        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distFront;

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
                LWPowerPU = -gamepad2.left_trigger;
                RWPowerPU = gamepad2.right_trigger;
                armPower = -gamepad2.left_stick_y;
                pickUpPwr = 0.7;
                //liftPower = -gamepad1.right_stick_y;

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
                // Controlling the amr location
                if (gamepad2.dpad_right)
                    robot.armPos(armLow,armEPower);
                else if (gamepad2.dpad_up)
                    robot.armPos(armHigh,1);
                else if (gamepad2.dpad_down)
                    robot.armPos(armLift,1);
                else
                    robot.armPos(armIdle,armEPower);
/*
                switch (armPosition) {
                    case ArmIdle:
                        if (gamepad2.dpad_left)
                            robot.armPos(armIdle,armEPower);
                    break;
                    case ArmLow:
                        if (gamepad2.dpad_right)
                            robot.armPos(armLow,armEPower);
                    break;
                    case ArmHigh:
                        if (gamepad2.dpad_up) {
                            armTimer.reset();
                            if(Math.abs(robot.leftArm.getCurrentPosition() - armHigh)<10)
                                robot.armPos(armHigh,1);
                        }
                    break;
                    case ArmLift:
                        if (gamepad2.dpad_down)
                            robot.armPos(armLift,1);
                    break;
                    //default:
                      //  armPosition = ArmPosition.ArmIdle;
                }

                //robot.moveArm(armPower);

                switch (pickupState) {
                    case PickupStart:
                        if (gamepad2.right_stick_y < 0.1 && gamepad2.right_stick_y > -0.1 && (gamepad2.right_bumper = false))
                            robot.movePickUp(pickupIdle,pickUpPwr);
                    break;
                    case PickupPick:
                        if (gamepad2.right_bumper)
                            robot.movePickUp(pickupPick,pickUpPwr);
                    break;
                    case PickupPlaceLow:
                        if (gamepad2.right_stick_y < -0.1)
                            robot.movePickUp(pickupLow,Math.max(-gamepad2.right_stick_y,pickUpPwr));
                    break;
                    case PickupPlaceHigh:
                        if (gamepad2.right_stick_y > 0.1)
                            robot.movePickUp(pickupHigh,Math.max(gamepad2.right_stick_y,pickUpPwr));
                    break;
                    //default:
                      //  pickupState = PickupState.PickupPick;
                }
                //robot.pwrPickUp(pickUpPwr);

                switch (pickupWheels) { // Controlling the pixel pick-up with the dpad and buttons (individual)
                    case PickupLin:
                        if (gamepad2.left_trigger>0)
                            robot.setPickupPower(LWPowerPU, 0);
                    break;
                    case PickupLout:
                        if (gamepad2.x)
                            robot.setPickupPower(0.3, 0);
                    break;
                    case PickupRin:
                        if (gamepad2.right_trigger>0)
                            robot.setPickupPower(0, RWPowerPU);
                    break;
                    case PickRout:
                        if (gamepad2.y)
                            robot.setPickupPower(0, -0.3);
                    break;
                    case PickupIdle:
                            robot.setPickupPower(0, 0);
                    break;
                    //default:
                      //  pickupWheels = PickupWheels.PickupIdle;
                }
*/
                if (gamepad1.dpad_up)
                    robot.moveLiftEnc(1250);
                else if (gamepad1.dpad_down)
                    robot.moveLiftEnc(0);

                servpos = robot.drone.getPosition();

                //Controlling the pickup location
                if (gamepad2.left_bumper)
                    robot.movePickUp(pickupPick, 0.7);
                else if (gamepad2.right_stick_y < -0.1)
                    robot.movePickUp(pickupLow,Math.max(-gamepad2.right_stick_y,pickUpPwr));
                else if (gamepad2.right_stick_y > 0.1)
                    robot.movePickUp(pickupHigh,Math.max(gamepad2.right_stick_y,pickUpPwr));
                else
                    robot.movePickUp(pickupIdle,0.5);

                // Controlling the pixel pick-up with the dpad and buttons (individual)
                if (gamepad2.left_trigger>0) {
                    robot.setPickupPower(LWPowerPU, 0);
                }
                else if (gamepad2.x) {
                    robot.setPickupPower(0.3, 0);
                }

                if (gamepad2.right_trigger>0) {
                    robot.setPickupPower(0, RWPowerPU);
                }
                else if (gamepad2.y) {
                    robot.setPickupPower(0, -0.3);
                }
/*
                //Controlling the arm to three specific positions - backdrop, drive, pickup
                if (gamepad2.dpad_up)
                    robot.armPos(160, 0.95); //Backdrop location
                else if (gamepad2.dpad_down)
                    robot.armPos(0,0.2); //Pickup location
                else if (gamepad2.dpad_right)
                    robot.armPos(45,0.9); //Drive location
*/

                //Drone launch
                if (gamepad2.options)
                    robot.setDronePosition(droneLaunch);

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
                telemetry.addData("PickUp Position", robot.pickUp.getCurrentPosition());
                telemetry.addData("Arm Power", armPower);
                telemetry.addData("Arm Position", robot.rightArm.getCurrentPosition());
                telemetry.addData("Arm Target", robot.rightArm.getTargetPosition());
                telemetry.addData("Drone", servpos);
                telemetry.addData("Lift Position", robot.lift.getCurrentPosition());

                telemetry.update();

                // Pace this loop so jaw action is reasonable speed.
                sleep(50);

            }
        }
    }
}
