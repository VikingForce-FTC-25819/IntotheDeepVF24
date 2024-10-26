package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.AutonDirection;

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
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void teleOpDrive(double drive, double turn, double powerFactor) {
        teleOpDrive(drive, turn, 0, powerFactor);

    }

    public void teleOpDrive(double drive, double turn, double strafe, double powerFactor) {
        strafe = strafe * 1.1; // Counteract imperfect strafing
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = powerFactor * (drive + strafe + turn) / denominator;
        double backLeftPower = powerFactor * (drive - strafe + turn) / denominator;
        double frontRightPower = powerFactor * (drive - strafe - turn) / denominator;
        double backRightPower = powerFactor * (drive + strafe - turn) / denominator;

        // Send calculated power to wheels
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);

    }

    public void autoDrive(int inches, double speed, AutonDirection autonDirection) {

        double ticksToTravel = inches * encTicksPerInches;
        DcMotor sensorToUse;
        // for reverse we need to reverse the speed
        switch (autonDirection) {
            case left:
                frontRight.setPower(speed);
                backLeft.setPower(speed);
                frontLeft.setPower(-speed);
                backRight.setPower(-speed);
                sensorToUse = perpendicularSensor;
                break;
            case right:
                frontRight.setPower(-speed);
                backLeft.setPower(-speed);
                frontLeft.setPower(speed);
                backRight.setPower(speed);
                sensorToUse = perpendicularSensor;
                break;
            case forward:
                frontRight.setPower(speed);
                backLeft.setPower(speed);
                frontLeft.setPower(speed);
                backRight.setPower(speed);
                sensorToUse = parallelSensor;
                break;
            case reverse:
                frontRight.setPower(-speed);
                backLeft.setPower(-speed);
                frontLeft.setPower(-speed);
                backRight.setPower(-speed);
                sensorToUse = parallelSensor;
                break;
            default:
                throw new IllegalArgumentException("autonDirection is not defined: " + autonDirection);
        }
        // Reset the encoder on the proper deadwheel
        sensorToUse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sensorToUse.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime runtime = new ElapsedTime();

        // our odometry pod is reversed in orientation so going forward gives a negative value, while
        // going backward gives a positive value - using the absolute value here allows us to ignore this.
        while (Math.abs(sensorToUse.getCurrentPosition()) <= ticksToTravel) {
            telemetry.addData("Distance", "traveled: %4.1f Inches", sensorToUse.getCurrentPosition() / encInchesPerTicks);
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