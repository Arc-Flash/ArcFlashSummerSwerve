package org.firstinspires.ftc.teamcode.pathfinder;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AbsoluteAnalogEncoder;

import java.util.function.Supplier;
import me.wobblyyyy.pathfinder2.geometry.Angle;
import me.wobblyyyy.pathfinder2.kinematics.RelativeSwerveModuleState;
import me.wobblyyyy.pathfinder2.robot.components.Motor;


@Config
public class PFModule {



    private DcMotorEx driveMotor;
    private CRServo servo;
    private AbsoluteAnalogEncoder absoluteAnalogEncoder;




    public PFModule(DcMotorEx drive, CRServo turn, AbsoluteAnalogEncoder encoder) {
        servo = turn;
        driveMotor= drive;
        absoluteAnalogEncoder = encoder;
    }

    /**
     * Get the module's turn servo.
     *
     * @return the module's turn servo.
     */
    public CRServo turn() {
        return this.servo;
    }

    /**
     * Get the module's drive motor.
     *
     * @return the module's drive motor.
     */
    public DcMotorEx drive() {
        return this.driveMotor;
    }

    /**
     * Set a swerve module state to the swerve module. This will set power
     * to both the turn servo and drive motor.
     *
     * @param state the state to set to the turn module.
     */
    public void set(RelativeSwerveModuleState state) {
        servo.setPower(state.getTurn());
        driveMotor.setPower(state.getDrive());
    }

    /**
     * Get the angle at which the swerve module is currently facing.
     *
     * @return the angle the swerve module is currently facing.
     */
    public Angle getAngle() {
        return new Angle( 360 * absoluteAnalogEncoder.getCurrentPosition() / 4096);
    }
}