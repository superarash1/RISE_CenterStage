package org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Mechanisms;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Control.MotionProfiling.TrapezoidalMotionProfile;
import org.firstinspires.ftc.team8109_Rise.Control.PID_Controller;
import org.firstinspires.ftc.team8109_Rise.Hardware.Drivetrains.MecanumDriveTrain;
import org.firstinspires.ftc.team8109_Rise.Math.Vectors.Vector3D;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Sensors.IMU;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Sensors.Odometry.OdometryLocalizer;
import org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Sensors.BeefCake_DriveConstants;

public class Chassis extends MecanumDriveTrain {
    Gamepad gamepad1;
    Telemetry telemetry;

    IMU imu;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.101399867722112;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1.15;
    public static double ω_WEIGHT = 1;

    public double fLeft;
    public double fRight;
    public double bLeft;
    public double bRight;

    public double odoDrive;
    public double odoStrafe;
    public double odoTurn;

    double max;

    double x_rotated;
    double y_rotated;

    public double trapezoidalTranslationalError = 0;

    static double PID_TranslationalX_kp = 0.3;
    static double PID_TranslationalY_kp = 0.3;
    static double PID_Heading_kp = 2.5;

    static double PID_TranslationalX_ki = 0.0015; //0.0075
    static double PID_TranslationalY_ki = 0.0015;
    static double PID_Heading_ki = 0.1; //0.05

    // 0.01
    static double PID_TranslationalX_kd = 0.045;
    static double PID_TranslationalY_kd = 0.045;
    static double PID_Heading_kd = 0.03;

    static double PID_TranslationalX_a = 0;
    static double PID_TranslationalY_a = 0;
    static double PID_Heading_a = 0.1;

    static double TrapezoidalX_kp = 0;
    static double TrapezoidalX_kv = 0.0075;
    static double TrapezoidalX_ka = 0;

    static double TrapezoidalY_kp = 0;
    static double TrapezoidalY_kv = 0;
    static double TrapezoidalY_ka = 0;

    double previousFLeft = 0;
    double previousFRight = 0;
    double previousBRight = 0;
    double previousBLeft = 0;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(BeefCake_DriveConstants.MAX_VEL, BeefCake_DriveConstants.MAX_ANG_VEL, BeefCake_DriveConstants.TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(BeefCake_DriveConstants.MAX_ACCEL);

    Vector3D controllerInput = new Vector3D(0, 0, 0);

    public PID_Controller TranslationalPID_X;
    public PID_Controller TranslationalPID_Y;
    public PID_Controller HeadingPID;

    public TrapezoidalMotionProfile TranslationalProfile_X;
    public TrapezoidalMotionProfile TranslationalProfile_Y;

    Vector3D odoPID_Vector = new Vector3D(0, 0, 0);

    public OdometryLocalizer odometry;

    public Vector3D RobotPose;

    public Chassis(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super("fLeft", "fRight", "bRight", "bLeft",
                BeefCake_DriveConstants.kV, BeefCake_DriveConstants.kA, BeefCake_DriveConstants.kStatic,
                BeefCake_DriveConstants.TRACK_WIDTH, BeefCake_DriveConstants.WHEEL_BASE, LATERAL_MULTIPLIER,
                TRANSLATIONAL_PID, HEADING_PID, VX_WEIGHT, VY_WEIGHT, ω_WEIGHT,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT, hardwareMap);

        reset();

        // TODO: Tune properly (needs some derivative)
        TranslationalPID_X = new PID_Controller(PID_TranslationalX_kp, PID_TranslationalX_kd, PID_TranslationalX_a, PID_TranslationalX_ki);//12.5 volts, a = 0
        TranslationalPID_Y = new PID_Controller(PID_TranslationalY_kp, PID_TranslationalY_kd, PID_TranslationalY_a, PID_TranslationalY_ki);
        HeadingPID = new PID_Controller(PID_Heading_kp, PID_Heading_kd, PID_Heading_a, PID_Heading_ki);

        TranslationalProfile_X = new TrapezoidalMotionProfile(BeefCake_DriveConstants.MAX_VEL, BeefCake_DriveConstants.MAX_ACCEL, TrapezoidalX_kp, TrapezoidalX_kv, TrapezoidalX_ka);
        TranslationalProfile_Y = new TrapezoidalMotionProfile(BeefCake_DriveConstants.MAX_VEL, BeefCake_DriveConstants.MAX_ACCEL, TrapezoidalY_kp, TrapezoidalY_kv, TrapezoidalY_ka);

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
        controllerInput.set(Math.pow(gamepad1.left_stick_y, 3), Math.pow(gamepad1.left_stick_x, 3), Math.pow(gamepad1.right_stick_x, 3));
//        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsRobotCentric(controllerInput);
    }

    public void fieldCentricTest(){
        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsFieldCentric(controllerInput);
    }

    public void chassisTelemetry(){

    }
}
