package org.firstinspires.ftc.robotcontroller.Hardware.Sensors;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public abstract class DriveConstants_Mecanum {

    public DriveConstants_Mecanum(){
        TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);
        TICKS_PER_DEGREE = (TICKS_PER_REV / 360);
        TICKS_PER_RADIAN = (TICKS_PER_REV / (2*Math.PI));
        WHEEL_RADIUS = WHEEL_DIAMETER/2;
    }

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public double TICKS_PER_REV;
    public double MAX_RPM;

    public static double GB_MECANUM_DIAMETER = 3.7795275590551181102362204724409;

    public double WHEEL_DIAMETER;
    public double TICKS_PER_INCH;
    public double TICKS_PER_RADIAN;
    public double TICKS_PER_DEGREE;
    public double NANOSECONDS_PER_MIN = 6e+10;
    public double LATERAL_MULTIPLIER;

    public PIDCoefficients TRANSLATIONAL_PID;
    public PIDCoefficients HEADING_PID;
    public double VX_WEIGHT;
    public double VY_WEIGHT;
    public double OMEGA_WEIGHT;

    public double LATERAL_DISTANCE; // in; distance between the left and right wheels 11.08
    public double FORWARD_OFFSET; // in; offset of the lateral wheel

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public boolean RUN_USING_ENCODER;
    public PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(25, 0, 7, 10.16785);

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public double WHEEL_RADIUS; // in
    public double GEAR_RATIO; // output (wheel) speed / input (motor) speed
    public double TRACK_WIDTH; // in
    public double WHEEL_BASE;

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public double kV;
    public double kA;
    public double kStatic;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    public double MAX_VEL;
    public double MAX_ACCEL;
    public double MAX_ANG_VEL;
    public double MAX_ANG_ACCEL;
}
