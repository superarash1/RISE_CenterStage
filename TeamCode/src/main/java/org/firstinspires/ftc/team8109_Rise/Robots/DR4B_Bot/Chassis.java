package org.firstinspires.ftc.team8109_Rise.Robots.DR4B_Bot;

import static org.firstinspires.ftc.team8109_Rise.Robots.DR4B_Bot.Sensors.DR4B_Bot_DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.team8109_Rise.Robots.DR4B_Bot.Sensors.DR4B_Bot_DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.team8109_Rise.Robots.DR4B_Bot.Sensors.DR4B_Bot_DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.team8109_Rise.Robots.DR4B_Bot.Sensors.DR4B_Bot_DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Hardware.Drivetrains.MecanumDriveTrain;
import org.firstinspires.ftc.team8109_Rise.Math.Vectors.Vector3D;
import org.firstinspires.ftc.team8109_Rise.Robots.DR4B_Bot.Sensors.DR4B_Bot_DriveConstants;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Sensors.IMU;

public class Chassis extends MecanumDriveTrain {

    Gamepad gamepad1;
    Telemetry telemetry;

    public IMU imu;
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.101399867722112;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double ω_WEIGHT = 1;

    double fLeft;
    double fRight;
    double bLeft;
    double bRight;
    double max;

    double x_rotated;
    double y_rotated;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    Vector3D controllerInput = new Vector3D(0, 0, 0);

    public Chassis(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super("fLeft", "fRight", "bRight", "bLeft",
                DR4B_Bot_DriveConstants.kV, DR4B_Bot_DriveConstants.kA, DR4B_Bot_DriveConstants.kStatic,
                DR4B_Bot_DriveConstants.TRACK_WIDTH, DR4B_Bot_DriveConstants.WHEEL_BASE, LATERAL_MULTIPLIER,
                TRANSLATIONAL_PID, HEADING_PID, VX_WEIGHT, VY_WEIGHT, ω_WEIGHT,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT, hardwareMap);

        imu = new IMU(hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void setDriveVectorsRobotCentric(Vector3D input){
        // Inverse Kinematics Calculations
//        fLeft = VY_WEIGHT * input.B - VX_WEIGHT * input.A - ω_WEIGHT * input.C;
//        fRight = VY_WEIGHT * input.B + VX_WEIGHT * input.A + ω_WEIGHT * input.C;
//        bRight = VY_WEIGHT * input.B - VX_WEIGHT * input.A + ω_WEIGHT * input.C;
//        bLeft = VY_WEIGHT * input.B + VX_WEIGHT * input.A - ω_WEIGHT * input.C;

        fLeft = VX_WEIGHT * input.A - VY_WEIGHT * input.B - ω_WEIGHT * input.C;
        fRight = VX_WEIGHT * input.A + VY_WEIGHT * input.B + ω_WEIGHT * input.C;
        bRight = VX_WEIGHT * input.A - VY_WEIGHT * input.B + ω_WEIGHT * input.C;
        bLeft = VX_WEIGHT * input.A + VY_WEIGHT * input.B - ω_WEIGHT * input.C;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }
        setPower(fLeft, fRight, bRight, bLeft);
    }

    public void setDriveVectorsFieldCentric(Vector3D input){
        x_rotated = input.A * Math.cos(imu.Angle_FieldCentric()) - input.B * Math.sin(imu.Angle_FieldCentric());
        y_rotated = input.A * Math.sin(imu.Angle_FieldCentric()) + input.B * Math.cos(imu.Angle_FieldCentric());

//        x_rotated = input.A * Math.cos(getPoseEstimate().getHeading()) - input.B * Math.sin(getPoseEstimate().getHeading());
//        y_rotated = input.A * Math.sin(getPoseEstimate().getHeading()) + input.B * Math.cos(getPoseEstimate().getHeading());

        // Inverse Kinematics Calculations
//        fLeft = VY_WEIGHT * y_rotated + VX_WEIGHT * x_rotated + ω_WEIGHT * input.C;
//        fRight = VY_WEIGHT * y_rotated - VX_WEIGHT * x_rotated - ω_WEIGHT * input.C;
//        bRight = VY_WEIGHT * y_rotated + VX_WEIGHT * x_rotated - ω_WEIGHT * input.C;
//        bLeft = VY_WEIGHT * y_rotated - VX_WEIGHT * x_rotated + ω_WEIGHT * input.C;

        fLeft = VX_WEIGHT * x_rotated - VY_WEIGHT * y_rotated - ω_WEIGHT * input.C;
        fRight = VX_WEIGHT * x_rotated + VY_WEIGHT * y_rotated + ω_WEIGHT * input.C;
        bRight = VX_WEIGHT * x_rotated - VY_WEIGHT * y_rotated + ω_WEIGHT * input.C;
        bLeft = VX_WEIGHT * x_rotated + VY_WEIGHT * y_rotated - ω_WEIGHT * input.C;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        setPower(fLeft, fRight, bRight, bLeft);
    }

    public void ManualDrive(){
        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsRobotCentric(controllerInput);
    }
}
