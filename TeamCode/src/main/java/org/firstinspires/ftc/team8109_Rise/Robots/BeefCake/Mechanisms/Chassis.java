package org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Control.MotionProfiling.TrapezoidalMotionProfile;
import org.firstinspires.ftc.team8109_Rise.Control.PID_Controller;
import org.firstinspires.ftc.team8109_Rise.Hardware.Drivetrains.MecanumDriveTrain;
import org.firstinspires.ftc.team8109_Rise.Math.Vectors.Vector3D;
import org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Sensors.IMU;
import org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Sensors.Odometry.OdometryLocalizer;
import org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Sensors.BeefCake_DriveConstants;

public class Chassis extends MecanumDriveTrain {
    Gamepad gamepad1;
    Telemetry telemetry;

    IMU imu;

    public static double LATERAL_MULTIPLIER = 1.101399867722112;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double ω_WEIGHT = 1;

    public Chassis(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super("fLeft", "fRight", "bRight", "bLeft",
                BeefCake_DriveConstants.kV, BeefCake_DriveConstants.kA, BeefCake_DriveConstants.kStatic,
                BeefCake_DriveConstants.TRACK_WIDTH, BeefCake_DriveConstants.WHEEL_BASE, LATERAL_MULTIPLIER,
                TRANSLATIONAL_PID, HEADING_PID, VX_WEIGHT, VY_WEIGHT, ω_WEIGHT, hardwareMap);

        reset();

        // TODO: Tune properly (needs some derivative)
        TranslationalPID_X = new PID_Controller(0, 0, 0, 0);//12.5 volts, a = 0
        TranslationalPID_Y = new PID_Controller(0, 0, 0, 0);
        HeadingPID = new PID_Controller(0, 0, 0, 0);

        TranslationalProfile_X = new TrapezoidalMotionProfile(BeefCake_DriveConstants.MAX_VEL, BeefCake_DriveConstants.MAX_ACCEL, 0, 0, 0);
        TranslationalProfile_Y = new TrapezoidalMotionProfile(BeefCake_DriveConstants.MAX_VEL, BeefCake_DriveConstants.MAX_ACCEL, 0, 0, 0);

        TranslationalPID_X.tolerance = 0.05;
        TranslationalPID_Y.tolerance = 0.05;

        TranslationalProfile_X.tolerance = 0.25;
        TranslationalProfile_Y.tolerance = 0.25;

        odometry = new OdometryLocalizer(hardwareMap);

        frontRight.setDirectionReverse();
        backRight.setDirectionReverse();
        frontLeft.setDirectionForward();
        backLeft.setDirectionForward();

        // if no 3 wheel odometry then remove
        setLocalizer(odometry);
        imu = new IMU(hardwareMap);

        RobotPose = new Vector3D(getPoseEstimate().getX(), getPoseEstimate().getY(), getPoseEstimate().getHeading());

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void setDriveVectorsRobotCentric(Vector3D input){
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
        x_rotated = input.A * Math.cos(getPoseEstimate().getHeading()) + input.B * Math.sin(getPoseEstimate().getHeading());
        y_rotated = input.A * Math.sin(getPoseEstimate().getHeading()) - input.B * Math.cos(getPoseEstimate().getHeading());

        fLeft = VX_WEIGHT * x_rotated + VY_WEIGHT * y_rotated - /*ω_WEIGHT*/ 1 * input.C;
        fRight = VX_WEIGHT * x_rotated - VY_WEIGHT * y_rotated + /*ω_WEIGHT*/ 1 * input.C;
        bRight = VX_WEIGHT * x_rotated + VY_WEIGHT * y_rotated + /*ω_WEIGHT*/ 1 * input.C;
        bLeft = VX_WEIGHT * x_rotated - VY_WEIGHT * y_rotated - /*ω_WEIGHT*/ 1 * input.C;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        if (fLeft != previousFLeft) frontLeft.setPower(fLeft);
        if (fRight != previousFRight) frontRight.setPower(fRight);
        if (bRight != previousBRight) backRight.setPower(bRight);
        if (bLeft != previousBLeft) backLeft.setPower(bLeft);

        setPower(fLeft, fRight, bRight, bLeft);

        previousFLeft = fLeft;
        previousFRight = fRight;
        previousBRight = bRight;
        previousBLeft = bLeft;
    }

    public void goToPosePID(Vector3D input){
        odoDrive = -TranslationalPID_X.PID_Power(getPoseEstimate().getX(), input.A);
        odoStrafe = -TranslationalPID_Y.PID_Power(getPoseEstimate().getY(), input.B);
        odoTurn = -HeadingPID.PID_Power(angleWrap(getPoseEstimate().getHeading()), input.C);

        odoPID_Vector.set(odoDrive, odoStrafe, odoTurn);

        setDriveVectorsFieldCentric(odoPID_Vector);
    }

    public void goToPoseTrapezoidal(Vector3D input){
        odoDrive = -TranslationalProfile_X.getProfilePower(getPoseEstimate().getX(), input.A);
        odoStrafe = -TranslationalProfile_Y.getProfilePower(getPoseEstimate().getY(), input.B);
        odoTurn = -HeadingPID.PID_Power(angleWrap(getPoseEstimate().getHeading()), input.C);

        odoPID_Vector.set(odoDrive, odoStrafe, odoTurn);

        trapezoidalTranslationalError = input.getVector2D().findDistance(getPoseVector().getVector2D());
        setDriveVectorsFieldCentric(odoPID_Vector);
    }

    public void ManualDrive(){
//        controllerInput.set(Math.pow(gamepad1.left_stick_y, 3), Math.pow(gamepad1.left_stick_x, 3), Math.pow(gamepad1.right_stick_x, 3));
        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsRobotCentric(controllerInput);
    }

    public void fieldCentricTest(){
        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsFieldCentric(controllerInput);
    }

    public void chassisTelemetry(){

    }
}
