package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;


@Config
public class SwerveModule {
    public static double P = 0.6, I = 0, D = 0.1;
    public static double K_STATIC = 0.03;

    public static double MAX_SERVO = 1, MAX_MOTOR = 0.2; //max speed of either, motor at 20% now for testing

    public static boolean MOTOR_FLIPPING = true;

    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1 / (1.5 * 2 * 2); // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 28;

    private DcMotorEx driveMotor;
    private CRServo servo;
    private AbsoluteAnalogEncoder absoluteAnalogEncoder;
    private PIDFController rotationController;

    public boolean wheelFlipped = false;
    private double targetAngle = 0.0;
    private double moduleAngle = 0.0;
    public double lastMotorPower = 0;

    public SwerveModule(DcMotorEx driveMotor, CRServo servo, AbsoluteAnalogEncoder absoluteAnalogEncoder) {
        this.driveMotor = driveMotor;
        MotorConfigurationType motorConfigurationType = this.driveMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        this.driveMotor.setMotorType(motorConfigurationType);
        this.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.servo = servo;
        ((CRServoImplEx) this.servo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));

        this.absoluteAnalogEncoder = absoluteAnalogEncoder;
        rotationController = new PIDFController(P, I, D, 0);
        this.driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public SwerveModule(HardwareMap hardwareMap, String motorName, String servoName, String absoluteAnalogEncoderName) {
        this(hardwareMap.get(DcMotorEx.class, motorName),
                hardwareMap.get(CRServo.class, servoName),
                new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, absoluteAnalogEncoderName)));
    }

    public void read() {
        moduleAngle = absoluteAnalogEncoder.getCurrentPosition();
    }

    public void update() {
        rotationController.setPIDF(P, I, D, 0);
        double rotationTarget = getTargetRotation(), currentAngle = getModuleRotation();

        double angleError = normalizeRadians(rotationTarget - currentAngle);
        if (MOTOR_FLIPPING && Math.abs(angleError) > Math.PI / 2) {
            rotationTarget = normalizeRadians(rotationTarget - Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }

        angleError = normalizeRadians(rotationTarget - currentAngle);

        double drivePower = Range.clip(rotationController.calculate(0, angleError), -MAX_SERVO, MAX_SERVO);
        if (Double.isNaN(drivePower)) drivePower = 0;
        servo.setPower(drivePower + (Math.abs(angleError) > 0.02 ? K_STATIC : 0) * Math.signum(drivePower));
    }

    public double getTargetRotation() {
        return normalizeRadians(targetAngle - Math.PI);
    }

    public double getModuleRotation() {
        return normalizeRadians(moduleAngle - Math.PI);
    }

    public void setMotorPower(double drivePower) {
        if (wheelFlipped) drivePower *= -1;
        lastMotorPower = drivePower;
        driveMotor.setPower(drivePower);
    }

    public void setTargetRotation(double targetAngle) {
        this.targetAngle = normalizeRadians(targetAngle);
    }

    public String getTelemetry(String moduleName) {
        return String.format(Locale.ENGLISH, "%s: Motor Flipped: %b \ncurrent position %.2f target position %.2f flip modifer = %d motor power = %.2f", moduleName, wheelFlipped, getModuleRotation(), getTargetRotation(), flipModifier(), lastMotorPower);
    }

    public int flipModifier() {
        return wheelFlipped ? -1 : 1;
    }


    public void setDriveMotorMode(DcMotor.RunMode runMode) {
        driveMotor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        driveMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        driveMotor.setPIDFCoefficients(runMode, coefficients);
    }


    public double getServoPower() {
        return servo.getPower();
    }

    public double getWheelPosition() {
        return encoderTicksToInches(driveMotor.getCurrentPosition());
    }

    public double getWheelVelocity() {
        return encoderTicksToInches(driveMotor.getVelocity());
    }

    public SwerveModuleState asState() {
        return new SwerveModuleState(this);
    }

    public static class SwerveModuleState {
        public SwerveModule module;
        public double wheelPosition, moduleRotation;

        public SwerveModuleState(SwerveModule s) {
            module = s;
            wheelPosition = 0;
            moduleRotation = 0;
        }

        public SwerveModuleState update() {
            return setState(-module.getWheelPosition(), module.getModuleRotation());
        }

        public SwerveModuleState setState(double wheelPosition, double moduleRotation) {
            this.wheelPosition = wheelPosition;
            this.moduleRotation = moduleRotation;
            return this;
        }

        //TODO add averaging for podrots based off of past values
        public Vector2d calculateDelta() {
            double oldWheelPosition = wheelPosition;
            update();
            return Vector2d.polar(wheelPosition - oldWheelPosition, moduleRotation);
        }
    }

    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}