package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.SwerveDrivetrain;

import java.util.function.BooleanSupplier;

@Config
@TeleOp(name = "😈😈😈😈😈😈")
public class Teleop extends CommandOpMode {
    private ElapsedTime timer;
    private double loopTime = 0;

    public static double position;

    private boolean pHeadingLock = true;
    private double targetHeading;

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;

    private SlewRateLimiter fw;
    private SlewRateLimiter str;
    private final PIDFController hController = new PIDFController(0.5, 0, 0.1, 0);

    public static double fw_r = 4;
    public static double str_r = 4;
    private boolean lock_robot_heading = false;

    GamepadEx gamepadEx, gamepadEx2;
    Localizer localizer;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Globals.AUTO = false;
        Globals.USING_IMU = true;
        Globals.USE_WHEEL_FEEDFORWARD = false;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

    }

    @Override
    public void run() {
        super.run();
        if (timer == null) {
            timer = new ElapsedTime();
            robot.startIMUThread(this);
            fw = new SlewRateLimiter(fw_r);
            str = new SlewRateLimiter(str_r);
        }

        robot.read(drivetrain);

        if (gamepad1.right_stick_button && Globals.USING_IMU)
            SwerveDrivetrain.imuOffset = robot.getAngle() + Math.PI;

        double turn = gamepad1.right_stick_x;
        if (Math.abs(turn) > 0.002) {
            lock_robot_heading = false;
        }

        double error = normalizeRadians(normalizeRadians(targetHeading) - normalizeRadians(robot.getAngle()));
        double headingCorrection = -hController.calculate(0, error) * 12.4 / robot.getVoltage();

        if (Math.abs(headingCorrection) < 0.01) {
            headingCorrection = 0;
        }

        SwerveDrivetrain.maintainHeading =
                (Math.abs(gamepad1.left_stick_x) < 0.002 &&
                        Math.abs(gamepad1.left_stick_y) < 0.002 &&
                        Math.abs(turn) < 0.002) &&
                        Math.abs(headingCorrection) < 0.02;


        double rotationAmount = (Globals.USING_IMU) ? robot.getAngle() - SwerveDrivetrain.imuOffset : 0;
        Pose drive = new Pose(
                new Point(joystickScalar(gamepad1.left_stick_y, 0.001),
                        joystickScalar(gamepad1.left_stick_x, 0.001)).rotate(rotationAmount),
                lock_robot_heading ? headingCorrection :
                        joystickScalar(turn, 0.01)
        );

        drive = new Pose(
                fw.calculate(drive.x),
                str.calculate(drive.y),
                drive.heading
        );


        robot.loop(drive, drivetrain);
        robot.write(drivetrain);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
    }

    private double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 4);
    }

    private double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }
}