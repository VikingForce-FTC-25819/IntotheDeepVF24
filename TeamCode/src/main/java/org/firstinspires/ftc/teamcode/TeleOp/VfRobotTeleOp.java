package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.ArmAngle;
import org.firstinspires.ftc.teamcode.Core.VfHardware;


/*
 * This is a teleop class in linearOpMode using a hardware abstraction to reduce the class complexity.
 */

@TeleOp(name="VF TeleOp", group="1")
public class VfRobotTeleOp extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // start at 50% power
    private double powerFactor = 0.5;

    /* multiplication factor used to control acceleration - a higher factor means faster
     * acceleration, a lower number means slower acceleration.  A factor higher than .5 accelerates
     * so fast that we have only two effective speeds because of the speed the loop runs in
     */
    public static final double ACCELERATION_FACTOR = 0.003;

    @Override
    public void runOpMode() {
        VfHardware robot = new VfHardware(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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

            if (gamepad1.y) {
                robot.autoLift();
            }

            if (gamepad2.a) {
                robot.collectSample();
            }

            if (gamepad2.b) {
                robot.deposit();
            }

            if (gamepad2.x) {
                robot.stopIntake();
            }

            if (gamepad2.dpad_up) {
                robot.raiseForHighSpecimenHang();
            }

            if (gamepad2.dpad_down) {
                robot.raiseToEnterSubmersible();
            }

            if (gamepad2.dpad_left) {
                robot.raiseForLowBasket();
            }

            if (gamepad2.dpad_up) {
                robot.raiseArmForLowHang();
            }

            if (gamepad2.y) {
                robot.liftRobot();
            }

//            if (gamepad2.right_bumper) {
//                robot.adjustArmAngle(ArmAngle.UP);
//            }
//
//            if (gamepad2.left_bumper) {
//                robot.adjustArmAngle(ArmAngle.DOWN);
//            }

            if (gamepad1.x) {
                robot.storeRobot();
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Parameters", "drive: %s, turn: %s, power: %s", drive, turn, powerFactor);
            telemetry.update();

        }
    }
}


