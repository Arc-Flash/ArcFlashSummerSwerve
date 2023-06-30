package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PIDController;

@TeleOp(name = "SwerveDriveTeleOp")
public class SwerveTeleOp extends LinearOpMode {
    // Declare drive motors
    private DcMotor motor1, motor2, motor3, motor4;

    // Declare module servos
    private Servo servo1, servo2, servo3, servo4;

    // Declare IMU
    private BNO055IMU imu;

    // Declare drive class
    private SampleMecanumDrive drive;

    // Declare PID controllers for each module
    private PIDController pidController1, pidController2, pidController3, pidController4;

    // Constants for module angles
    private static final double ANGLE_1 = 0.0;  // Adjust as needed
    private static final double ANGLE_2 = 0.0; // Adjust as needed
    private static final double ANGLE_3 = 0.0; // Adjust as needed
    private static final double ANGLE_4 = 0.0; // Adjust as needed

    // Constants for servo positions
    private static final double SERVO_MIN = 0.0; // Adjust as needed
    private static final double SERVO_MAX = 1.0; // Adjust as needed

    // Variables for joystick values
    private double driveX, driveY, rotate;

    // Variable for field-centric drive
    private boolean fieldCentricDrive = true;

    // Timer for updating module angles
    private ElapsedTime angleUpdateTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        drive = new SampleMecanumDrive(hardwareMap);

        // Initialize PID controllers
        pidController1 = new PIDController(0.0, 0.0, 0.0);
        pidController2 = new PIDController(0.0, 0.0, 0.0);
        pidController3 = new PIDController(0.0, 0.0, 0.0);
        pidController4 = new PIDController(0.0, 0.0, 0.0);

        // Set servo initial positions
        servo1.setPosition(SERVO_MIN);
        servo2.setPosition(SERVO_MIN);
        servo3.setPosition(SERVO_MIN);
        servo4.setPosition(SERVO_MIN);

        // Wait for start button to be pressed
        waitForStart();

        // Run until the end of the match
        while (opModeIsActive()) {
            // Update joystick values
            driveX = gamepad1.left_stick_x;
            driveY = -gamepad1.left_stick_y;
            rotate = -gamepad1.right_stick_x;

            // Convert joystick values to field-centric drive if enabled
            if (fieldCentricDrive) {
                double heading = Math.toRadians(getHeading());
                double temp = driveY * Math.cos(heading) - driveX * Math.sin(heading);
                driveX = driveY * Math.sin(heading) + driveX * Math.cos(heading);
                driveY = temp;
            }

            // Calculate module angles
            double angle1 = calculateModuleAngle(driveX, driveY, rotate, ANGLE_1);
            double angle2 = calculateModuleAngle(driveX, driveY, rotate, ANGLE_2);
            double angle3 = calculateModuleAngle(driveX, driveY, rotate, ANGLE_3);
            double angle4 = calculateModuleAngle(driveX, driveY, rotate, ANGLE_4);

            // Update module angles periodically to prevent servo jitter
            if (angleUpdateTimer.milliseconds() > 100) {
                servo1.setPosition(mapAngleToServo(angle1));
                servo2.setPosition(mapAngleToServo(angle2));
                servo3.setPosition(mapAngleToServo(angle3));
                servo4.setPosition(mapAngleToServo(angle4));
                angleUpdateTimer.reset();
            }

            // Set motor powers based on joystick values
            double power1 = calculateMotorPower(driveX, driveY, rotate, angle1);
            double power2 = calculateMotorPower(driveX, driveY, rotate, angle2);
            double power3 = calculateMotorPower(driveX, driveY, rotate, angle3);
            double power4 = calculateMotorPower(driveX, driveY, rotate, angle4);

            motor1.setPower(power1);
            motor2.setPower(power2);
            motor3.setPower(power3);
            motor4.setPower(power4);

            // Update telemetry
            telemetry.addData("Drive X", driveX);
            telemetry.addData("Drive Y", driveY);
            telemetry.addData("Rotate", rotate);
            telemetry.addData("Angle 1", angle1);
            telemetry.addData("Angle 2", angle2);
            telemetry.addData("Angle 3", angle3);
            telemetry.addData("Angle 4", angle4);
            telemetry.update();
        }
    }

    private double calculateModuleAngle(double driveX, double driveY, double rotate, double baseAngle) {
        return baseAngle - Math.toDegrees(Math.atan2(driveY, driveX)) + 90.0 - rotate;
    }

    private double calculateMotorPower(double driveX, double driveY, double rotate, double moduleAngle) {
        double error = moduleAngle - getModuleAngle(moduleAngle); // Calculate module angle error
        double pidOutput = 0.0;

        // Select PID controller based on module angle
        if (moduleAngle == ANGLE_1) {
            pidOutput = pidController1.calculate(error);
        } else if (moduleAngle == ANGLE_2) {
            pidOutput = pidController2.calculate(error);
        } else if (moduleAngle == ANGLE_3) {
            pidOutput = pidController3.calculate(error);
        } else if (moduleAngle == ANGLE_4) {
            pidOutput = pidController4.calculate(error);
        }

        // Scale joystick values and add PID output for motor power calculation
        return Range.clip(driveX + rotate - pidOutput, -1.0, 1.0);
    }

    private double getModuleAngle(double baseAngle) {
        // Retrieve module angle from potentiometer
        // Implement your own code here to read the absolute position from the Axon Mini+ servo's built-in potentiometer.
        // This depends on the specific library or hardware you're using to interface with the servo.

        // This is just a placeholder, assuming the servo returns the base angle as the module angle.
        return baseAngle;
    }

    private double mapAngleToServo(double moduleAngle) {
        // Map module angle to servo position
        double servoPosition = ((moduleAngle - ANGLE_1) / 360.0) * (SERVO_MAX - SERVO_MIN) + SERVO_MIN;
        return Range.clip(servoPosition, SERVO_MIN, SERVO_MAX);
    }

    private double getHeading() {
        // Get heading from IMU
        // Implement your own code here to read the heading from the BNO055IMU.
        // This depends on the specific library or hardware you're using to interface with the IMU.

        // This is just a placeholder, assuming the IMU returns a constant heading of 0.0.
        return 0.0;
    }
}
