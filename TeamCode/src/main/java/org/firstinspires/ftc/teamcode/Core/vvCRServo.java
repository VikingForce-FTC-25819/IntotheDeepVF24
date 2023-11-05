package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class vvCRServo extends LinearOpMode {
    private CRServo continuousServo;

    @Override
    public void runOpMode() {
        // Initialize the continuous servo by mapping it to the hardware configuration
        continuousServo = hardwareMap.get(CRServo.class, "servoName"); // Replace "servoName" with the name you've assigned to your servo

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Control the continuous servo
            double power = gamepad1.left_stick_y; // Use the gamepad left stick to control the servo
            continuousServo.setPower(power);

            // You can also use setPosition to control the servo position, but it won't stop at a specific angle as it's a continuous servo.
            // double position = 0.5 + 0.5 * gamepad1.left_stick_y; // Set position based on gamepad input
            // continuousServo.setPosition(position);

            telemetry.addData("Servo Power", power);
            telemetry.update();
        }
    }

}
