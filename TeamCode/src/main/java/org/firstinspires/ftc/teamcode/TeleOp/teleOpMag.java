
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
import org.firstinspires.ftc.teamcode.Core.vvHardwareMag;

/**
 * Mr. Price's teleOp for Magnus
 *
 * Need to confirm the drive system and add the drone servo to have a full test case
 * Also need the telemetry to read all sensor values
 */

@TeleOp(name="MagTeleOp", group="Concept")

public class teleOpMag extends LinearOpMode {

    //vvHardware class external pull
    vvHardwareMag   robot       = new vvHardwareMag(this);

    // An Enum is used to represent pickup state
    // (This is one thing enums are designed to do)
    public enum PickupState {
        PickupStart,
        PickupDrive,
        PickupPlaceLow,
        PickupPlaceHigh
    };
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
        double armEPower = 0;
        double pickUpPwr = 0;
        double droneSet = 0.25;
        double droneLaunch = 0;
        double servpos = 0;

        // The pickupState variable is declared out here
        // so its value persists between loop() calls
        PickupState pickupState = PickupState.PickupStart;

        // used with the dump servo, this will get covered in a bit
        ElapsedTime pickupTimer = new ElapsedTime();

        final int pickupIdle; // the idle position for the pickup motor
        final int pickupHigh; // the placing position for the pickup motor in the high position
        final int pickupLow; // the placing position for the pickup motor in the low/forward position

        // the amount of time the pickup takes to activate in seconds
        final double pickupTime;
        // the amount of time the arm takes to raise in seconds
        final double armTime;

        final int armLow; // the low encoder position for the arm
        final int armHigh; // the high-overhead encoder position for the arm


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
                pickUpPwr = -gamepad2.right_stick_y * 0.75;
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
/*
                robot.moveArm(armPower);

                robot.pwrPickUp(pickUpPwr);

                if (gamepad1.dpad_up)
                    robot.moveLiftEnc(1250);
                else if (gamepad1.dpad_down)
                    robot.moveLiftEnc(0);

                servpos = robot.drone.getPosition();
/*
                //Controlling the pickup location
                if (gamepad2.left_bumper)
                    robot.movePickUp(45, 0.7);
                else if (gamepad2.right_bumper)
                    robot.movePickUp(-90,0.4);
               // else if (gamepad1.right_stick_button)
                   // robot.movePickUp(0,0.5);
                else
                    robot.movePickUp(0,0.5);

                //Controlling the arm to three specific positions - backdrop, drive, pickup
                if (gamepad2.dpad_up)
                    robot.armPos(160, 0.95); //Backdrop location
                else if (gamepad2.dpad_down)
                    robot.armPos(0,0.2); //Pickup location
                else if (gamepad2.dpad_right)
                    robot.armPos(45,0.9); //Drive location

                // Controlling the pixel pick-up with the dpad and buttons (individual)
                if (gamepad2.left_trigger>0) {
                    robot.setPickupPower(LWPowerPU, 0);
                } else if (gamepad2.right_trigger>0) {
                    robot.setPickupPower(0, RWPowerPU);
                } else if (gamepad2.y)
                    robot.setPickupPower(0, -0.3);
                else if (gamepad2.x)
                    robot.setPickupPower(0.3, 0);
                else {
                    robot.setPickupPower(0, 0);
                }

                switch (pickupState) {
                    case PickupStart:
                        // Waiting for some input
                        if (gamepad1.x) {
                            // x is pressed, start extending
                            liftMotor.setTargetPosition(LIFT_HIGH);
                            liftState = LiftState.LIFT_EXTEND;
                        }
                        break;
                    case LIFT_EXTEND:
                        // check if the lift has finished extending,
                        // otherwise do nothing.
                        if (Math.abs(liftMotor.getCurrentPosition() - LIFT_HIGH) < 10) {
                            // our threshold is within
                            // 10 encoder ticks of our target.
                            // this is pretty arbitrary, and would have to be
                            // tweaked for each robot.

                            // set the lift dump to dump
                            liftDump.setTargetPosition(DUMP_DEPOSIT);

                            liftTimer.reset();
                            liftState = LiftState.LIFT_DUMP;
                        }
                        break;
                    case LIFT_DUMP:
                        if (liftTimer.seconds() >= DUMP_TIME) {
                            // The robot waited long enough, time to start
                            // retracting the lift
                            liftDump.setTargetPosition(DUMP_IDLE);
                            liftMotor.setTargetPosition(LIFT_LOW);
                            liftState = LiftState.LIFT_RETRACT;
                        }
                        break;
                    case LIFT_RETRACT:
                        if (Math.abs(liftMotor.getCurrentPosition() - LIFT_LOW) < 10) {
                            liftState = LiftState.LIFT_START;
                        }
                        break;
                    default:
                        // should never be reached, as liftState should never be null
                        liftState = LiftState.LIFT_START;
                }

                // small optimization, instead of repeating ourselves in each
                // lift state case besides LIFT_START for the cancel action,
                // it's just handled here
                if (gamepad1.y && liftState != LiftState.LIFT_START) {
                    liftState = LiftState.LIFT_START;
                }

                // mecanum drive code goes here
                // But since none of the stuff in the switch case stops
                // the robot, this will always run!
                updateDrive(gamepad1, gamepad2);

                //Drone launch
                if (gamepad2.options)
                    robot.setDronePosition(droneLaunch);
*/
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

                telemetry.update();

                // Pace this loop so jaw action is reasonable speed.
                sleep(50);

            }
        }
    }
}
