package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VfHardware {
    private final OpMode opMode;
    private final Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor parallelSensor;
    private DcMotor perpendicularSensor;

    public static double WHEEL_DIAMETER = 1.88976; // in
    public static final double TICKS_PER_REV = 2000;
    public double encTicksPerInches = TICKS_PER_REV/(WHEEL_DIAMETER*Math.PI);
    public double encInchesPerTicks = (WHEEL_DIAMETER*Math.PI)/TICKS_PER_REV;

    public VfHardware(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
        // Initialize the drive system variables.
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        this.backLeft = hardwareMap.get(DcMotor.class, "RLM");
        this.backRight = hardwareMap.get(DcMotor.class, "RRM");
        this.frontLeft = hardwareMap.get(DcMotor.class, "FLM");
        this.frontRight = hardwareMap.get(DcMotor.class, "FRM");
        this.parallelSensor = frontLeft;
        this.perpendicularSensor = backLeft;

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // because we have four motors we need set all 4 motor directions
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection((DcMotorSimple.Direction.REVERSE));
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void teleOpDrive(double drive, double turn, double powerFactor) {
        double frontLeftPower   = powerFactor * (Range.clip(drive + turn, -1.0, 1.0)) ;
        double frontRightPower  = powerFactor * (Range.clip(drive - turn, -1.0, 1.0)) ;
        double backLeftPower    = powerFactor * (Range.clip(drive + turn, -1.0, 1.0)) ;
        double backRightPower   = powerFactor * (Range.clip(drive - turn, -1.0, 1.0)) ;

        // Send calculated power to wheels
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);

    }

    public void autoDriveForward(int inches, double speed) {
        parallelSensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelSensor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // these 2 lines needed to zero out the odometry sensor
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
        ElapsedTime runtime = new ElapsedTime();
        // our odometry pod is reversed in orientation so going forward gives a negative value
        // this is why we need to use a negative sign in our comparison
        // we want to drive forwards until our current position is larger than our desired forward distance
        // so we drive until our current position is no longer <= our desired distance
        while (-parallelSensor.getCurrentPosition() <= inches * encTicksPerInches) {
            telemetry.addData("Distance", "Forward: %4.1f Inches", -parallelSensor.getCurrentPosition()/encInchesPerTicks);
            telemetry.addData("Path", "%4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void autoDrive(int inches, double speed, boolean reverse) {
        double ticksToTravel = inches * encTicksPerInches;
        // for reverse we need to reverse the speed and ticks to travel
        if (reverse) {
            speed = -speed;
        }
        parallelSensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelSensor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // these 2 lines needed to zero out the odometry sensor
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
        ElapsedTime runtime = new ElapsedTime();

        // our odometry pod is reversed in orientation so going forward gives a negative value, while
        // going backward gives a positive value - using the absolute value here allows us to ignore this.
        while (Math.abs(parallelSensor.getCurrentPosition()) <= ticksToTravel) {
            telemetry.addData("Distance", "Forward: %4.1f Inches", -parallelSensor.getCurrentPosition() / encInchesPerTicks);
            telemetry.addData("Path", "%4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void autoDriveBackward(int inches, double speed) {
        parallelSensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelSensor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // these 2 lines needed to zero out the odometry sensor
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);
        ElapsedTime runtime = new ElapsedTime();
        // our odometry pod is reversed in orientation so going backward gives a positive value
        // on the parallel sensor - this is why we need to use a pos sign in our comparison
        // we want to drive backwards until our current position is larger than our desired backward distance
        // so we drive until our current position is no longer <= our desired distance
        while (parallelSensor.getCurrentPosition() <= inches * encTicksPerInches) {
            telemetry.addData("Distance", "Forward: %4.1f Inches", -parallelSensor.getCurrentPosition() / encInchesPerTicks);
            telemetry.addData("Path", "%4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void pause(double pauseTimeSeconds) {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < pauseTimeSeconds) {
            telemetry.addData("Initial Pause", "Paused: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}