package org.firstinspires.ftc.team8109_Rise.Sensors.Odometry;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcontroller.Hardware.Sensors.DriveConstants_Mecanum;

public class TenacityDriveConstants extends DriveConstants_Mecanum {
    public TenacityDriveConstants(){
        TICKS_PER_REV = 537.7;
        MAX_RPM = 312;

        WHEEL_DIAMETER = GB_MECANUM_DIAMETER;

        VX_WEIGHT = 1;
        VY_WEIGHT = 1;
        OMEGA_WEIGHT = 1;


        GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
        TRACK_WIDTH = 19.07; // in
        WHEEL_BASE = 19.07;

        LATERAL_DISTANCE = 11.081; // in; distance between the left and right wheels 11.08
        FORWARD_OFFSET = 4.5; // in; offset of the lateral wheel

        /*
         * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
         * Set this flag to false if drive encoders are not present and an alternative localization
         * method is in use (e.g., tracking wheels).
         *
         * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
         * from DriveVelocityPIDTuner.
         */
        RUN_USING_ENCODER = false;

        MOTOR_VELO_PID = new PIDFCoefficients(25, 0, 7, 10.16785);
        TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
        HEADING_PID = new PIDCoefficients(0, 0, 0);

        LATERAL_MULTIPLIER = 0;

        kV = 0;
        kA = 0;
        kStatic = 0;

        MAX_VEL = 0;
        MAX_ACCEL = 0;
        MAX_ANG_VEL = 0;
        MAX_ANG_ACCEL = 0;
    }
}