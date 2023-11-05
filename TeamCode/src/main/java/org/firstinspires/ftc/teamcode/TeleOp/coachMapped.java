
package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.vvHardware;

/**
 * Mr. Price's teleOp for test and explanation - this one uses a Hardware Class structure
 *
 * Need to confirm the drive system and add the drone servo to have a full test case
 * Also need the telemetry to read all sensor values
 */

@TeleOp(name="coachMap", group="1-TeleOp")

public class coachMapped extends LinearOpMode {

    //vvHardware class external pull
    vvHardware   robot       = new vvHardware(this);

    public ColorSensor colorSensor;
    public DistanceSensor distFront;
    public DistanceSensor distRear;
@Override
    public void runOpMode() throws InterruptedException {
        double driveY = 0;
        double strafe = 0;
        double turn = 0;
        double RWPower = 0;
        double LWPower = 0;
        double RWPowerPU = 0;
        double LWPowerPU = 0;
        double drivePower = 0.5; //global drive power level
        double armPower = 0;
        double droneSet = 0.25;
        double droneLaunch = 0;
        double servpos = 0;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        //float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        //final float values[] = hsvValues;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

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
                strafe = gamepad1.left_stick_x * -1;
                turn = gamepad1.right_stick_x;
                LWPowerPU = gamepad2.left_trigger;
                RWPowerPU = gamepad2.right_trigger;
                armPower = -gamepad2.left_stick_y;

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

                robot.moveArm(armPower);

                servpos = robot.drone.getPosition();

                //Controlling the arm to three specific positions - backdrop, drive, pickup
                if (gamepad2.dpad_up)
                    robot.armPos(160, 0.95); //Backdrop location
                else if (gamepad2.dpad_down)
                    robot.armPos(0,0.4); //Pickup location
                else if (gamepad2.dpad_right)
                    robot.armPos(25,0.9); //Drive location

                // Controlling the pixel pick-up with the dpad and buttons (individual)
                if (gamepad2.left_trigger>0) {
                    robot.setPickupPower(LWPowerPU, 0);
                } else if (gamepad2.right_trigger>0) {
                    robot.setPickupPower(0, -RWPowerPU);
                } else if (gamepad2.y)
                    robot.setPickupPower(-0.2, 0);
                else if (gamepad2.x)
                    robot.setPickupPower(0, 0.2);
                else {
                    robot.setPickupPower(0, 0);
                }
                //Drone launch
                if (gamepad2.left_bumper)
                    robot.setDronePosition(droneLaunch);

// Adding telemetry readouts
                telemetry.addData(">", "Robot Running");
                telemetry.addData("Drive Power", drivePower);
                telemetry.addData("Y", driveY);
                telemetry.addData("strafe", strafe);
                telemetry.addData("turn", turn);
                telemetry.addData("Arm Power", armPower);
                telemetry.addData("Arm Position", robot.rightArm.getCurrentPosition());
                telemetry.addData("Arm Target", robot.rightArm.getTargetPosition());
                telemetry.addData("Drone", servpos);

/* check the status of the x button on either gamepad.
                bCurrState = gamepad1.x;

                // check for button state transitions.
                if (bCurrState && (bCurrState != bPrevState)) {

                    // button is transitioning to a pressed state. So Toggle LED
                    bLedOn = !bLedOn;
                    colorSensor.enableLed(bLedOn);
                }

                // update previous state variable.
                bPrevState = bCurrState;
*/
                // convert the RGB values to HSV values.
                /*Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

                // send the info back to driver station using telemetry function.
                telemetry.addData("Clear", colorSensor.alpha());
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());
                telemetry.addData("Hue", hsvValues[0]);

                telemetry.addData("FDS", distFront.getDeviceName());
                telemetry.addData("range", String.format("%.01f cm", distFront.getDistance(DistanceUnit.CM)));
                telemetry.addData("range", String.format("%.01f in", distFront.getDistance(DistanceUnit.INCH)));

                telemetry.addData("RDS", distRear.getDeviceName());
                telemetry.addData("range", String.format("%.01f cm", distRear.getDistance(DistanceUnit.CM)));
                telemetry.addData("range", String.format("%.01f in", distRear.getDistance(DistanceUnit.INCH)));

                // Rev2mDistanceSensor specific methods.
                telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
                telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
*/
                telemetry.update();

                // Use gamepad buttons to move arm up (Y) and down (A)
            /*if (gamepad1.y)
                leftArm.setPower(ARM_UP_POWER);
            else if (gamepad1.a)
                leftArm.setPower(ARM_DOWN_POWER);
            else
                leftArm.setPower(0.0);
            */
                // Send telemetry message to signify robot running;
               /* telemetry.addData("claw", "Offset = %.2f", clawOffset);
                telemetry.addData("left", "%.2f", left);
                telemetry.addData("right", "%.2f", right);
                telemetry.update();
*/
                // Pace this loop so jaw action is reasonable speed.
                sleep(50);

            }
        }
    }
}
