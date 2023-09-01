package org.firstinspires.ftc.teamcode.pathfinder;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.Locale;

import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.odometrycore.ThreeWheelOdometry;
import me.wobblyyyy.pathfinder2.robot.AbstractOdometry;
import me.wobblyyyy.pathfinder2.robot.Odometry;
import me.wobblyyyy.pathfinder2.robot.sensors.Encoder;


/**
 * An example implementation of {@link PFOdometry}, which uses
 * {@code OdometryCore} to track a robot's position.
 *
 * @since 2.4.0
 */
public class PFOdometry extends AbstractOdometry {
    // PLACEHOLDER!!!!
    private static final double CPR = 4096;
    private static final double WHEEL_DIAMETER = 0.0;
    private static final double OFFSET_LEFT = 0.0;
    private static final double OFFSET_RIGHT = 0.0;
    private static final double OFFSET_CENTER = 0.0;

    // PLACEHOLDER!!!!!
    private Motor.Encoder leftEncoder;
    private Motor.Encoder rightEncoder;
    private Motor.Encoder centerEncoder;

    /**
     * Create a new instance of the {@code Robot} class to demonstrate how
     * {@link PFOdometry} is instantiated.
     */

        public PFOdometry(RobotHardware robotMap){
            leftEncoder = robotMap.parallelPod;
            rightEncoder = robotMap.perpindicularPod;
            centerEncoder = robotMap.centerPod;
        }




        // create a ThreeWheelOdometryProfile to store constants
        ThreeWheelOdometry.ThreeWheelOdometryProfile odometryProfile = new ThreeWheelOdometry.ThreeWheelOdometryProfile(
                CPR,
                WHEEL_DIAMETER,
                OFFSET_LEFT,
                OFFSET_RIGHT,
                OFFSET_CENTER
        );


        ThreeWheelOdometry.EncoderProfile encoderProfile = new ThreeWheelOdometry.EncoderProfile(
                () -> (double) leftEncoder.getPosition(),
                () -> (double) rightEncoder.getPosition(),
                () -> (double) centerEncoder.getPosition()
        );

        // initialize ThreeWheelOdometry
        Odometry odometry = new ThreeWheelOdometry(
                odometryProfile,
                encoderProfile
        );


    @Override
    public PointXYZ getRawPosition() {
        PointXYZ position = odometry.getPosition();
        return position;
    }

    public String getTelemetry(){
        return String.format(Locale.ENGLISH,"X Position: .2%f \nY Position: .2%f \nZ Position: .2%f ", getX(),getY(),getZ());

    }
}

