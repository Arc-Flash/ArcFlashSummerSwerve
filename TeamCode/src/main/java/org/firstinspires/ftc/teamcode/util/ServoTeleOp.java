package org.firstinspires.ftc.teamcode.util; // Update with your actual package name

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTeleOp")
public class ServoTeleOp extends OpMode {
    private Servo servo;
    private double servoPosition;
    private boolean buttonAPressed;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo"); // "servo" should be replaced with your servo's configured name
        servoPosition = 0.5; // Initial servo position (usually 0.5 is centered)
        buttonAPressed = false;
    }

    @Override
    public void loop() {
        // Check if button A is pressed on gamepad 1
        if (gamepad1.a && !buttonAPressed) {
            // Move the servo by 20 degrees
            servoPosition += 20.0 / 180.0; // 20 degrees in servo position units (assuming servo range is 0 to 1)
            buttonAPressed = true;
        } else if (!gamepad1.a) {
            buttonAPressed = false;
        }

        // Keep servo position within valid range (usually 0.0 to 1.0)
        servoPosition = Math.min(1.0, Math.max(0.0, servoPosition));

        // Set the servo position
        servo.setPosition(servoPosition);

        // Other TeleOp code here
        // ...

        // Update telemetry (if needed)
        telemetry.addData("Servo Position", servoPosition);
        telemetry.update();
    }
}