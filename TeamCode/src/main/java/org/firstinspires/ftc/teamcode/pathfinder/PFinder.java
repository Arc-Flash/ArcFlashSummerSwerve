package org.firstinspires.ftc.teamcode.pathfinder;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.SwerveModule;

import me.wobblyyyy.pathfinder2.robot.Drive;

import java.util.ArrayList;
import java.util.List;
import me.wobblyyyy.pathfinder2.Pathfinder;
import me.wobblyyyy.pathfinder2.control.Controller;
import me.wobblyyyy.pathfinder2.control.GenericTurnController;
import me.wobblyyyy.pathfinder2.follower.FollowerGenerator;
import me.wobblyyyy.pathfinder2.follower.generators.GenericFollowerGenerator;
import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.geometry.Translation;
import me.wobblyyyy.pathfinder2.robot.Odometry;
import me.wobblyyyy.pathfinder2.robot.Robot;
import me.wobblyyyy.pathfinder2.robot.simulated.SimulatedDrive;
import me.wobblyyyy.pathfinder2.robot.simulated.SimulatedOdometry;
import me.wobblyyyy.pathfinder2.trajectory.Trajectory;
import me.wobblyyyy.pathfinder2.trajectory.builder.LinearTrajectoryBuilder;

public class PFinder {

    private static final double SPEED = 0.5;
    private static final double TOLERANCE = 1.0;
    private static final Angle ANGLE_TOLERANCE = Angle.fromDeg(15);

    private final RobotHardware robotMap;
    private final Controller turnController = new GenericTurnController(0.1);
    private final FollowerGenerator followerGenerator = new GenericFollowerGenerator(
            turnController
    );
    private final Drive drive;
    private final Odometry odometry;
    private final Robot robot;
    private final Pathfinder pathfinder;

    public PFinder(RobotHardware robotMap) {

        this.robotMap = robotMap;

        drive = new PFDrive(robotMap, turnController, 0.1);
        odometry = new PFOdometry(robotMap);
        robot = new Robot(drive, odometry);
        pathfinder = new Pathfinder(
                robot,
                followerGenerator
        );

        pathfinder.setSpeed(SPEED);
        pathfinder.setTolerance(TOLERANCE);
        pathfinder.setAngleTolerance(ANGLE_TOLERANCE);
    }



    public void teleopDrive(GamepadEx gamepadEx){

        while (true) {
            double x = gamepadEx.getLeftX();
            double y = gamepadEx.getLeftY();
            double z = gamepadEx.getRightX();

            Translation translation = new Translation(x, y, z);

            Translation absoluteTranslation = translation.toRelative(Angle.fromRad(robotMap.getAngle()));

            pathfinder.setTranslation(absoluteTranslation);
        }

    }

    private void goToPoint(PointXYZ target) {
        pathfinder.clear();

        pathfinder.goTo(target);

        while (pathfinder.isActive()) {
            // commented out to support jdk8... :(
            // Thread.onSpinWait();

            pathfinder.tick();
        }
    }
    public void runTrajectory( Trajectory trajectory) {

        pathfinder.followTrajectory(trajectory);
    }
    public void runTrajectories (  List<Trajectory> trajectories){

        pathfinder.followTrajectories(trajectories);
    }
    public void loop() {

        pathfinder.tick();
    }
}
