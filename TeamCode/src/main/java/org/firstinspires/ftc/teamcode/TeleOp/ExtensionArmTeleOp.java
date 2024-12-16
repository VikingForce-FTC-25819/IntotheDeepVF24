package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.Random;


/*
 * This is a teleop class in linearOpMode using a hardware abstraction to reduce the class complexity.
 */

@TeleOp(name="Extension Arm TeleOp", group="1")
public class ExtensionArmTeleOp extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // start at 50% power
    private double powerFactor = 0.75;

    /* multiplication factor used to control acceleration - a higher factor means faster
     * acceleration, a lower number means slower acceleration.  A factor higher than .5 accelerates
     * so fast that we have only two effective speeds because of the speed the loop runs in
     */
    public static final double ACCELERATION_FACTOR = 0.003;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, false);

//        if (this.enableArmChaos()) {
//            robot.disableArm();
//        }
//
//        if (this.enableChassisChaos()) {
//            robot.disableChassis();
//        }


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            // give the driver ability to reset the arm angle to 0
            if (gamepad1.start) {
                robot.resetArmAngle();
            }

            // give the arm diver ability to reset the slide 0
//            if (gamepad2.start) {
//                robot.resetSlide();
//            }
            // set up the left bumper button to decelerate and right bumper to accelerate
            if (gamepad1.left_bumper) {
                powerFactor = Math.max(powerFactor - ACCELERATION_FACTOR, 0.1);
            }
            if (gamepad1.right_bumper) {
                powerFactor = Math.min(powerFactor + ACCELERATION_FACTOR, 1.0);
            }

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;

            robot.teleOpDrive(drive, turn, strafe, powerFactor);

//            if (gamepad1.y) {
//                robot.autoLift();
//            }

            if (gamepad2.a) {
                robot.collectSample();
            }

            if (gamepad2.b) {
                robot.deposit();
            }

            if (gamepad2.x) {
                robot.closeClaw();
            }

            if (gamepad2.dpad_left) {
                robot.raiseForHighBasket();
            }

            if (gamepad2.dpad_up) {
                robot.raiseArmForLowHang();
            }

            if (gamepad2.dpad_down) {
                robot.raiseForSpecimenCollect();
            }

            if (gamepad2.y) {
                robot.liftRobot();
            }

            robot.adjustArmAngle(gamepad2.right_trigger + (-gamepad2.left_trigger));

            robot.adjustArmAngleContinuous(gamepad2.left_stick_y);

            robot.adjustSlideContinuous(gamepad2.right_stick_y);

            robot.adjustWristAngleContinuous(gamepad2.right_stick_x);

            if (gamepad1.x) {
                robot.storeRobot();
            }

            telemetry.addData("trigger values: ", "%4.2f -- %4.2f", gamepad2.right_trigger, gamepad2.left_trigger);
            telemetry.update();

        }
    }

    private boolean enableArmChaos() {
        if (beChaotic()) {
            telemetry.addData("Arm Chaos Enabled", "Good Luck!!");
            return true;
        } else {
            return false;
        }
    }

    private boolean enableChassisChaos() {
        if (beChaotic()) {
            telemetry.addData("Chassis Chaos Enabled", "Good Luck!!");
            return true;
        } else {
            return false;
        }
    }

    private boolean beChaotic() {
        Random random = new Random();
        int randomNumber = random.nextInt(3); // Generates a random number between 0 (inclusive) and 3 (exclusive)
        return randomNumber == 0; // Returns true if the random number is 0 (which happens 1/5 of the time)
    }
}


