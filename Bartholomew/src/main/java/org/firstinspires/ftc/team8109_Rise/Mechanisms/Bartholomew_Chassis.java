package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Control.MotionProfiling.TrapezoidalMotionProfile;
import org.firstinspires.ftc.robotcontroller.Control.PID_Controller;
import org.firstinspires.ftc.robotcontroller.Hardware.Drivetrains.MecanumDriveTrain;
import org.firstinspires.ftc.robotcontroller.Hardware.SlidesBot_DriveConstants;
import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Sensors.Odometry.Bartholomew_DriveConstants;
import org.firstinspires.ftc.team8109_Rise.Sensors.Odometry.Bartholomew_OdometryLocalizer;

// make extends MecanumDriveTrain_Old if using drive encoders
@Config
public class Bartholomew_Chassis extends MecanumDriveTrain {

    Gamepad gamepad1;
    Telemetry telemetry;

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

    Vector3D controllerInput = new Vector3D(0, 0, 0);

    public PID_Controller TranslationalPID_X;
    public PID_Controller TranslationalPID_Y;
    public PID_Controller HeadingPID;

    public TrapezoidalMotionProfile TranslationalProfile_X;
    public TrapezoidalMotionProfile TranslationalProfile_Y;

    Vector3D odoPID_Vector = new Vector3D(0, 0, 0);

    public Vector3D RobotPose;

    static String[] names = {"fLeft", "fRight", "bRight", "bLeft"};

    public Bartholomew_Chassis(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super(names, new Bartholomew_DriveConstants(),
                OdometryType.THREE_WHEEL_ODO, new Bartholomew_OdometryLocalizer(hardwareMap), hardwareMap);

        reset();

        // TODO: Tune properly (needs some derivative)
        TranslationalPID_X = new PID_Controller(PID_TranslationalX_kp, PID_TranslationalX_kd, PID_TranslationalX_a, PID_TranslationalX_ki);//12.5 volts, a = 0
        TranslationalPID_Y = new PID_Controller(PID_TranslationalY_kp, PID_TranslationalY_kd, PID_TranslationalY_a, PID_TranslationalY_ki);
        HeadingPID = new PID_Controller(PID_Heading_kp, PID_Heading_kd, PID_Heading_a, PID_Heading_ki);

        TranslationalProfile_X = new TrapezoidalMotionProfile(SlidesBot_DriveConstants.MAX_VEL, SlidesBot_DriveConstants.MAX_ACCEL, TrapezoidalX_kp, TrapezoidalX_kv, TrapezoidalX_ka);
        TranslationalProfile_Y = new TrapezoidalMotionProfile(SlidesBot_DriveConstants.MAX_VEL, SlidesBot_DriveConstants.MAX_ACCEL, TrapezoidalY_kp, TrapezoidalY_kv, TrapezoidalY_ka);

        TranslationalPID_X.tolerance = 0.05;
        TranslationalPID_Y.tolerance = 0.05;

        TranslationalProfile_X.tolerance = 0.25;
        TranslationalProfile_Y.tolerance = 0.25;

        frontRight.setDirectionReverse();
        backRight.setDirectionReverse();
        frontLeft.setDirectionForward();
        backLeft.setDirectionForward();

        RobotPose = new Vector3D(getPoseEstimate().getX(), getPoseEstimate().getY(), getPoseEstimate().getHeading());

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void setDriveVectorsFieldCentric(Vector3D input){
        x_rotated = input.A * Math.cos(getPoseEstimate().getHeading()) + input.B * Math.sin(getPoseEstimate().getHeading());
        y_rotated = input.A * Math.sin(getPoseEstimate().getHeading()) - input.B * Math.cos(getPoseEstimate().getHeading());

        fLeft = CONSTANTS.VX_WEIGHT * x_rotated + CONSTANTS.VY_WEIGHT * y_rotated - CONSTANTS.OMEGA_WEIGHT * input.C;
        fRight = CONSTANTS.VX_WEIGHT * x_rotated - CONSTANTS.VY_WEIGHT * y_rotated + CONSTANTS.OMEGA_WEIGHT * input.C;
        bRight = CONSTANTS.VX_WEIGHT * x_rotated + CONSTANTS.VY_WEIGHT * y_rotated + CONSTANTS.OMEGA_WEIGHT * input.C;
        bLeft = CONSTANTS.VX_WEIGHT * x_rotated - CONSTANTS.VY_WEIGHT * y_rotated - CONSTANTS.OMEGA_WEIGHT * input.C;

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
        odoDrive = -TranslationalProfile_X.getPosProfilePower(getPoseEstimate().getX(), input.A);
        odoStrafe = -TranslationalProfile_Y.getPosProfilePower(getPoseEstimate().getY(), input.B);
        odoTurn = -HeadingPID.PID_Power(angleWrap(getPoseEstimate().getHeading()), input.C);

        odoPID_Vector.set(odoDrive, odoStrafe, odoTurn);

        trapezoidalTranslationalError = input.getVector2D().findDistance(getPoseVector().getVector2D());
        setDriveVectorsFieldCentric(odoPID_Vector);
    }

    public void fieldCentricTest(){
        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsFieldCentric(controllerInput);
    }
}
