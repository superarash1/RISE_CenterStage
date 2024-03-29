package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Drivetrains.MecanumDriveTrain;
import org.firstinspires.ftc.robotcontroller.Hardware.SlidesBot_DriveConstants;
import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Sensors.CookieMonsterDriveConstants;

public class CookieMonster_Chassis extends MecanumDriveTrain {
    Gamepad gamepad1;
    Telemetry telemetry;

    public double fLeft;
    public double fRight;
    public double bLeft;
    public double bRight;

    double max;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.101399867722112;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1.15;
    public static double ω_WEIGHT = 1;

    public boolean lastToggleA = false;
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(SlidesBot_DriveConstants.MAX_VEL, SlidesBot_DriveConstants.MAX_ANG_VEL, SlidesBot_DriveConstants.TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(SlidesBot_DriveConstants.MAX_ACCEL);

    public enum DriveDirection{
        FORWARD,
        REVERSE
    }

    DriveDirection driveDirection;
    Vector3D controllerInput = new Vector3D(0, 0, 0);

    static String[] names = {"fLeft", "fRight", "bRight", "bLeft"};
    public CookieMonster_Chassis(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super(names, new CookieMonsterDriveConstants(),
                OdometryType.THREE_WHEEL_ODO, hardwareMap);

        reset();

        frontRight.setDirectionReverse();
        backRight.setDirectionReverse();
        frontLeft.setDirectionForward();
        backLeft.setDirectionForward();

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        driveDirection = DriveDirection.FORWARD;
    }

    public void setDriveVectorsRobotCentric(Vector3D input){
        //TODO: Test cube weighting
        // Inverse Kinematics Calculations
        fLeft = VX_WEIGHT * input.A - VY_WEIGHT * input.B - ω_WEIGHT * input.C;
        fRight = VX_WEIGHT * input.A + VY_WEIGHT * input.B + ω_WEIGHT * input.C;
        bRight = VX_WEIGHT * input.A - VY_WEIGHT * input.B + ω_WEIGHT * input.C;
        bLeft = VX_WEIGHT * input.A + VY_WEIGHT * input.B - ω_WEIGHT * input.C;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bRight /= max;
            bLeft /= max;
        }
        setPower(fLeft, fRight, bRight, bLeft);
    }

    public void setDriveVectorsRobotCentric_Invertible(Vector3D input){
        switch (driveDirection){
            case FORWARD:
                fLeft = VX_WEIGHT * input.A - VY_WEIGHT * input.B - ω_WEIGHT * input.C;
                fRight = VX_WEIGHT * input.A + VY_WEIGHT * input.B + ω_WEIGHT * input.C;
                bRight = VX_WEIGHT * input.A - VY_WEIGHT * input.B + ω_WEIGHT * input.C;
                bLeft = VX_WEIGHT * input.A + VY_WEIGHT * input.B - ω_WEIGHT * input.C;

                if ((gamepad1.a != lastToggleA) && gamepad1.a){
                    driveDirection = DriveDirection.REVERSE;
                }
                break;
            case REVERSE:
                fLeft = -VX_WEIGHT * input.A + VY_WEIGHT * input.B - ω_WEIGHT * input.C;
                fRight = -VX_WEIGHT * input.A - VY_WEIGHT * input.B + ω_WEIGHT * input.C;
                bRight = -VX_WEIGHT * input.A + VY_WEIGHT * input.B + ω_WEIGHT * input.C;
                bLeft = -VX_WEIGHT * input.A - VY_WEIGHT * input.B - ω_WEIGHT * input.C;

                if ((gamepad1.a != lastToggleA) && gamepad1.a){
                    driveDirection = DriveDirection.FORWARD;
                }
                break;
        }

        lastToggleA = gamepad1.a;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bRight /= max;
            bLeft /= max;
        }
        setPower(fLeft, fRight, bRight, bLeft);
    }

    public void ManualDrive(){
        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsRobotCentric_Invertible(controllerInput);
    }

    public void Telemetry(){
        telemetry.addData("Drive Mode", driveDirection);
    }
}
