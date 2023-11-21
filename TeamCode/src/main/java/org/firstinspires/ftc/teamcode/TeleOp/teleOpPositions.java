
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
 * Copy of coachMapped except pickup positions are paired with the arm positions
 *
 * Need to confirm the drive system and add the drone servo to have a full test case
 * Also need the telemetry to read all sensor values
 */

@TeleOp(name="teleOpPosition", group="1-TeleOp")

public class teleOpPositions extends LinearOpMode {

    //vvHardware class external pull
    vvHardware   robot       = new vvHardware(this);

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
        int encX = 0;
        int encY = 0;
        double RWPower = 0;
        double LWPower = 0;
        double RWPowerPU = 0;
        double LWPowerPU = 0;
        double drivePower = 0.7; //global drive power level
        double armPower = 0;
        double liftPower = 0;
        double armEPower = 0.8;
        double pickUpPwr = 0.7;
        double droneSet = 0.25;
        double droneLaunch = 0;
        double servpos = 0;

        // The pickupState variable is declared out here
        // so its value persists between loop() calls
        PickupState pickupState = PickupState.PickupStart;

        // used with the dump servo, this will get covered in a bit
        ElapsedTime pickupTimer = new ElapsedTime();

        final int pickupIdle = 1; // the idle position for the pickup motor 109
        final int pickupHigh = 28; // the placing position for the pickup motor in the high position 148
        final int pickupLow = 5; // the placing position for the pickup motor in the low/forward position 5

        // the amount of time the pickup takes to activate in seconds
        final double pickupTime;
        // the amount of time the arm takes to raise in seconds
        final double armTime;

        final int armIdle = 0; // -84
        final int armLow = 100; // the low encoder position for the arm -23
        final int armHigh = 401; // the high-overhead encoder position for the arm 329
        final int armHang = 470;


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
                //pickUpPwr = -gamepad2.right_stick_y * 0.5;
                //liftPower = -gamepad1.right_stick_y;

                encY = -robot.parallelEncoder.getCurrentPosition(); //need to reverse
                encX = robot.perpendicularEncoder.getCurrentPosition();

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

                //robot.moveLift(liftPower);
                if (gamepad1.dpad_up)
                    robot.moveLiftEnc(1250);
                else if (gamepad1.dpad_down)
                    robot.moveLiftEnc(0);

                servpos = robot.drone.getPosition();

                //Controlling the pickup location
                if (gamepad2.x)
                    robot.movePickUp(pickupIdle, pickUpPwr);
                else if (gamepad2.b)
                    robot.movePickUp(pickupHigh,pickUpPwr);
                else if (gamepad2.a)
                    robot.movePickUp(pickupLow,pickUpPwr);
                //else if (gamepad2.y)
                  //  robot.movePickUp(pickupIdle,pickUpPwr);

                //Controlling the arm to three specific positions - backdrop, drive, pickup
                if (gamepad2.dpad_up) {
                    robot.armPos(armHigh, armEPower); //Backdrop location
                    robot.movePickUp(pickupHigh,pickUpPwr);}
                else if (gamepad2.dpad_down){
                    robot.armPos(armIdle,armEPower); //Pickup location
                    robot.movePickUp(pickupIdle, pickUpPwr);}
                else if (gamepad2.dpad_right){
                    robot.armPos(armLow,armEPower); //Drive location
                    robot.movePickUp(pickupLow,pickUpPwr);}
                else if (gamepad2.dpad_left)
                   robot.armPos(armHang,armEPower);

                // Controlling the pixel pick-up with the trigger and buttons (individual)
                if (gamepad2.left_trigger>0)
                    robot.leftWheel.setPower(LWPowerPU);
                else if (gamepad2.left_bumper)
                    robot.leftWheel.setPower(1);
                else
                    robot.leftWheel.setPower(0);

                if (gamepad2.right_trigger>0)
                    robot.rightWheel.setPower(RWPowerPU);
                else if (gamepad2.right_bumper)
                    robot.rightWheel.setPower(-1);
                else
                    robot.rightWheel.setPower(0);

                //if (gamepad2.right_trigger>0) {
                  //  robot.setRightClawPosition(vvHardware.clawClose);
                //} else if (gamepad2.right_bumper)
                  //  robot.setRightClawPosition(vvHardware.clawOpen);

                //Drone launch
                if (gamepad2.options)
                    robot.setDronePosition(droneLaunch);

                // Retrieve Rotational Angles and Velocities
                YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES);

// Adding telemetry readouts
                telemetry.addData(">", "Robot Running");
                telemetry.addData("Drive Power", drivePower);
                //telemetry.addData("Y", driveY);
                //telemetry.addData("strafe", strafe);
                //telemetry.addData("turn", turn);
                telemetry.addData("Y Encoder",encY);
                telemetry.addData("X Encoder",encX);
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("PickUp Position", robot.pickUp.getCurrentPosition());
                //telemetry.addData("Arm Power", armPower);
                telemetry.addData("Arm Position", robot.rightArm.getCurrentPosition());
                telemetry.addData("Arm Target", robot.rightArm.getTargetPosition());
                //telemetry.addData("Claw Position", robot.rightClaw.getPosition());
                telemetry.addData("Drone", servpos);
                telemetry.addData("Lift Position", robot.lift.getCurrentPosition());

                telemetry.update();

                // Pace this loop
                sleep(20);

            }
        }
    }
}
