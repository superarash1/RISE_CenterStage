package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Control.MotionProfiling.TrapezoidalMotionProfile;
import org.firstinspires.ftc.team8109_Rise.Control.PIDF_Controller;
import org.firstinspires.ftc.team8109_Rise.Hardware.Drivetrains.MecanumDriveTrain;
import org.firstinspires.ftc.team8109_Rise.Math.Vectors.Vector2D;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Sensors.IMU;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Sensors.Odometry.OdometryLocalizer;
import org.firstinspires.ftc.team8109_Rise.Math.Vectors.Vector3D;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Sensors.SlidesBot_DriveConstants;
import org.firstinspires.ftc.team8109_Rise.Sensors.Camera.OpenCV.VisionPipelines.PowerPlayPipeline;

@Config
public class Chassis extends MecanumDriveTrain {

    Gamepad gamepad1;
    Telemetry telemetry;

    ElapsedTime runtime = new ElapsedTime();

    IMU imu;

    public double kDrift = 0.1;
    public enum TrackingObject{
        CONESTACK,
        JUNCTION,
        NONE
    }

    public enum Alliance{
        RED,
        BLUE
    }

    public TrackingObject trackingObject;
    public Alliance alliance;
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.101399867722112;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1.15;
    public static double ω_WEIGHT = 1; //0.75

    public double targetHeading;

    public double driftCorrection;
    public double fLeft;
    public double fRight;
    public double bLeft;
    public double bRight;

    public double odoDrive;
    public double odoStrafe;
    public double odoTurn;

    double visionTurn;
    double visionDrive;

    double max;

    double x_rotated;
    double y_rotated;

    public double trapezoidalTranslationalError = 0;

    // 0.06
//    static double translationalX_kp = 0.2;
//    static double translationalY_kp = 0.13;
//    static double heading_kp = 2.5;
//
//    static double visionX_kp = 0;
//    static double visionHeading_kp = 0;
//
//    static double translationalX_ki = 0.002; //0.0075
//    static double translationalY_ki = 0.002;
//    //TODO: Slightly Decrease
//    static double heading_ki = 0.1; //0.05
//
//    // 0.01
//    static double translationalX_kd = 0.02;
//    static double translationalY_kd = 0.025;
//    static double heading_kd = 0.045;

//
//    static double translationalX_kp = 0.275;
//    static double translationalY_kp = 0.25;
//    static double heading_kp = 4;
//
//    static double visionX_kp = 0;
//    static double visionHeading_kp = 0;
//
//    static double translationalX_ki = 0.003; //0.0075
//    static double translationalY_ki = 0.0025;
//    //TODO: Slightly Decrease
//    static double heading_ki = 0; //0.05
//
//    // 0.01
//    static double translationalX_kd = 0.04;
//    static double translationalY_kd = 0.04;
//    static double heading_kd = 0.05;

//    static double translationalX_kp = 0.2;
//    static double translationalY_kp = 0.2;
//    static double heading_kp = 2.5;
//
//    static double visionX_kp = 0;
//    static double visionHeading_kp = 0;
//
//    static double translationalX_ki = 0.002; //0.0075
//    static double translationalY_ki = 0.002;
//    //TODO: Slightly Decrease
//    static double heading_ki = 0.1; //0.05
//
//    // 0.01
//    static double translationalX_kd = 0.05;
//    static double translationalY_kd = 0.05;
//    static double heading_kd = 0.045;


    // W tunings
//    static double translationalX_kp = 0.325;
//    static double translationalY_kp = 0.325;
//    static double heading_kp = 2.5;
//
//    static double visionX_kp = 0;
//    static double visionHeading_kp = 0;
//
//    static double translationalX_ki = 0.002; //0.0075
//    static double translationalY_ki = 0.002;
//    //TODO: Slightly Decrease
//    static double heading_ki = 0.1; //0.05
//
//    // 0.01
//    static double translationalX_kd = 0.05;
//    static double translationalY_kd = 0.05;
//    static double heading_kd = 0.03;

    //static double translationalX_kp = 0.325;
    //    static double translationalY_kp = 0.325;
    //    static double heading_kp = 2.5;
    //
    //    static double visionX_kp = 0;
    //    static double visionHeading_kp = 0;
    //
    //    static double translationalX_ki = 0; //0.0075
    //    static double translationalY_ki = 0;
    //    //TODO: Slightly Decrease
    //    static double heading_ki = 0.1; //0.05
    //
    //    // 0.01
    //    static double translationalX_kd = 0.04;
    //    static double translationalY_kd = 0.04;
    //    static double heading_kd = 0.03;

    //12.5 volt tunings
//    static double translationalX_kp = 0.325;
//    static double translationalY_kp = 0.325;
//    static double heading_kp = 2.5;
//
//    static double visionX_kp = 0;
//    static double visionHeading_kp = 0;
//
//    static double translationalX_ki = 0; //0.0075
//    static double translationalY_ki = 0;
//    //TODO: Slightly Decrease
//    static double heading_ki = 0.1; //0.05
//
//    // 0.01
//    static double translationalX_kd = 0.045;
//    static double translationalY_kd = 0.045;
//    static double heading_kd = 0.03;
//
//    static double translationalX_a = 0;
//    static double translationalY_a = 0;
//    static double Heading_a = 0.1;
    //_________________________________________

//    static double translationalX_kp = 0.315;
//    static double translationalY_kp = 0.315;
//    static double heading_kp = 2.5;
//
//    static double visionX_kp = 0;
//    static double visionHeading_kp = 0;
//
//    static double translationalX_ki = 0.001; //0.0075
//    static double translationalY_ki = 0.001;
//    //TODO: Slightly Decrease
//    static double heading_ki = 0.1; //0.05
//
//    // 0.01
//    static double translationalX_kd = 0.045;
//    static double translationalY_kd = 0.045;
//    static double heading_kd = 0.03;
//
//    static double translationalX_a = 0;
//    static double translationalY_a = 0;
//    static double Heading_a = 0.1;

    static double PID_TranslationalX_kp = 0.3;
    static double PID_TranslationalY_kp = 0.3;
    static double PID_Heading_kp = 2.5;

    static double PID_TranslationalX_ki = 0.0015; //0.0075
    static double PID_TranslationalY_ki = 0.0015;
    //TODO: Slightly Decrease
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

//    static double visionX_kp = 0.003;
//    static double visionHeading_kp = 0.0025;
//
//    static double visionX_kd = 0.0002; //0.0002
//    static double visionHeading_kd = 0.0002; //0.00015
//
//    static double visionX_ki = 0.001;
//    static double visionHeading_ki = 0.002;

    static double visionX_kp = 0.0025;
    static double visionHeading_kp = 0.0025;

    static double visionX_kd = 0; //0.0002
    static double visionHeading_kd = 0; //0.00015

    static double visionX_ki = 0.0025;
    static double visionHeading_ki = 0;

//    static double visionX_ki = 0.0017;
//    static double visionHeading_ki = 0.001;

    double previousFLeft = 0;
    double previousFRight = 0;
    double previousBRight = 0;
    double previousBLeft = 0;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(SlidesBot_DriveConstants.MAX_VEL, SlidesBot_DriveConstants.MAX_ANG_VEL, SlidesBot_DriveConstants.TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(SlidesBot_DriveConstants.MAX_ACCEL);

    Vector3D controllerInput = new Vector3D(0, 0, 0);

    public PIDF_Controller TranslationalPID_X;
    public PIDF_Controller TranslationalPID_Y;
    public PIDF_Controller HeadingPID;

    public PIDF_Controller visionX_PID;
    public PIDF_Controller visionHeading_PID;
    public PIDF_Controller IMU_PID;

    public TrapezoidalMotionProfile TranslationalProfile_X;
    public TrapezoidalMotionProfile TranslationalProfile_Y;

    Vector3D odoPID_Vector = new Vector3D(0, 0, 0);
    Vector3D visionPID_Vector = new Vector3D(0, 0, 0);
    Vector3D PID_Vector = new Vector3D(0, 0, 0);
    Vector3D powerInput = new Vector3D(0,0,0);

    Vector3D[] inputArray;

    Vector3D VisionAlignPID_Vector = new Vector3D(0, 0, 0);

    public OdometryLocalizer odometry;

    public Vector3D RobotPose;

//    public Chassis(ConeTracker coneTracker, JunctionTracker junctionPipeline, Telemetry telemetry, HardwareMap hardwareMap){
//        super("fLeft", "fRight", "bRight", "bLeft",
//                SlidesBot_DriveConstants.kV, SlidesBot_DriveConstants.kA, SlidesBot_DriveConstants.kStatic,
//                SlidesBot_DriveConstants.TRACK_WIDTH, SlidesBot_DriveConstants.WHEEL_BASE, LATERAL_MULTIPLIER,
//                TRANSLATIONAL_PID, HEADING_PID, VX_WEIGHT, VY_WEIGHT, ω_WEIGHT,
//                VEL_CONSTRAINT, ACCEL_CONSTRAINT, hardwareMap);
//
//        reset();
//
//        // TODO: Tune properly (needs some derivative
////        TranslationalPID_X = new PIDF_Controller(translationalX_kp, translationalX_kd, 0.8, translationalX_ki);
////        TranslationalPID_Y = new PIDF_Controller(translationalY_kp, translationalY_kd, 0.8, translationalY_ki);
////        HeadingPID = new PIDF_Controller(heading_kp, heading_kd, 0, heading_ki);
//
//        TranslationalPID_X = new PIDF_Controller(translationalX_kp, translationalX_kd, 0.95, translationalX_ki);
//        TranslationalPID_Y = new PIDF_Controller(translationalY_kp, translationalY_kd, 0.95, translationalY_ki);
//        HeadingPID = new PIDF_Controller(heading_kp, heading_kd, 0, heading_ki);
//
//        visionX_PID = new PIDF_Controller(visionX_kp);
//        visionHeading_PID = new PIDF_Controller(visionHeading_kp);
//
//        IMU_PID = new PIDF_Controller(2);
//
//        TranslationalPID_X.tolerance = 0.2;
//        TranslationalPID_Y.tolerance = 0.2;
//        HeadingPID.tolerance = 0.009;
//
//        IMU_PID.tolerance = 0.009;
//
//        odometry = new OdometryLocalizer(hardwareMap);
//
//        frontLeft.setDirectionReverse();
//        backLeft.setDirectionReverse();
//
//        frontLeft.setBreakMode();
//        frontRight.setBreakMode();
//        backRight.setBreakMode();
//        backLeft.setBreakMode();
//
//        setLocalizer(odometry);
//        imu = new IMU(hardwareMap);
////
////        this.junctionPipeline = junctionPipeline;
////        this.conePipeline = coneTracker;
//        this.telemetry = telemetry;
//
//        trackingObject = TrackingObject.NONE;
//    }

    public Chassis(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super("fLeft", "fRight", "bRight", "bLeft",
                SlidesBot_DriveConstants.kV, SlidesBot_DriveConstants.kA, SlidesBot_DriveConstants.kStatic,
                SlidesBot_DriveConstants.TRACK_WIDTH, SlidesBot_DriveConstants.WHEEL_BASE, LATERAL_MULTIPLIER,
                TRANSLATIONAL_PID, HEADING_PID, VX_WEIGHT, VY_WEIGHT, ω_WEIGHT,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT, hardwareMap);

        reset();

        // TODO: Tune properly (needs some derivative)
        TranslationalPID_X = new PIDF_Controller(PID_TranslationalX_kp, PID_TranslationalX_kd, PID_TranslationalX_a, PID_TranslationalX_ki);//12.5 volts, a = 0
        TranslationalPID_Y = new PIDF_Controller(PID_TranslationalY_kp, PID_TranslationalY_kd, PID_TranslationalY_a, PID_TranslationalY_ki);
        HeadingPID = new PIDF_Controller(PID_Heading_kp, PID_Heading_kd, PID_Heading_a, PID_Heading_ki);

        TranslationalProfile_X = new TrapezoidalMotionProfile(SlidesBot_DriveConstants.MAX_VEL, SlidesBot_DriveConstants.MAX_ACCEL, TrapezoidalX_kp, TrapezoidalX_kv, TrapezoidalX_ka);
        TranslationalProfile_Y = new TrapezoidalMotionProfile(SlidesBot_DriveConstants.MAX_VEL, SlidesBot_DriveConstants.MAX_ACCEL, TrapezoidalY_kp, TrapezoidalY_kv, TrapezoidalY_ka);

        visionX_PID = new PIDF_Controller(visionX_kp, visionX_kd, 0, visionX_ki);
        visionHeading_PID = new PIDF_Controller(visionHeading_kp, visionHeading_kd, 0, visionHeading_ki);

        TranslationalPID_X.tolerance = 0.05;
        TranslationalPID_Y.tolerance = 0.05;

        TranslationalProfile_X.tolerance = 0.25;
        TranslationalProfile_Y.tolerance = 0.25;

        visionX_PID.tolerance = 1;
        visionHeading_PID.tolerance = 1;

        odometry = new OdometryLocalizer(hardwareMap);

        frontRight.setDirectionReverse();
        backRight.setDirectionReverse();
        frontLeft.setDirectionForward();
        backLeft.setDirectionForward();

        setLocalizer(odometry);
        imu = new IMU(hardwareMap);

        RobotPose = new Vector3D(getPoseEstimate().getX(), getPoseEstimate().getY(), getPoseEstimate().getHeading());

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
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
            bLeft /= max;
            bRight /= max;
        }

        setPower(fLeft, fRight, bRight, bLeft);
    }

    public void setCorrectedDriveVectorsRobotCentric(Vector3D input){
        //TODO: Test cube weighting
        // Inverse Kinematics Calculations

        powerInput = input.add(IMU_Correction());
        fLeft = VX_WEIGHT * powerInput.A - VY_WEIGHT * powerInput.B - ω_WEIGHT * powerInput.C;
        fRight = VX_WEIGHT * powerInput.A + VY_WEIGHT * powerInput.B + ω_WEIGHT * powerInput.C;
        bRight = VX_WEIGHT * powerInput.A - VY_WEIGHT * powerInput.B + ω_WEIGHT * powerInput.C;
        bLeft = VX_WEIGHT * powerInput.A + VY_WEIGHT * powerInput.B - ω_WEIGHT * powerInput.C;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        setPower(fLeft, fRight, bRight, bLeft);
    }

    //TODO: May need to to negate drift and IMU PID
    public Vector3D IMU_Correction(){
        if (controllerInput.C == 0){
            targetHeading = imu.Angle_FieldCentric();
            driftCorrection = IMU_PID.PIDF_Power(imu.Angle_FieldCentric(), targetHeading) + controllerInput.B*kDrift;
        }else {
            driftCorrection = controllerInput.B*kDrift;
        }
        return new Vector3D(0,0, driftCorrection);
    }

    // Unrefined field centric code
    //TODO: Tune heading a bit more
    public void setDriveVectorsFieldCentric(Vector3D input){
        //TODO: Use IMU possibly
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

    public void  robotRelative(Pose2d powers) {
        setWeightedDrivePower(powers);
    }

//    public void fieldRelative(Pose2d powers) {
//
//        Vector2d vec = new Vector2d(powers.getX(),-powers.getY());
//
//        vec = vec.rotated(getPoseEstimate().getHeading());
//        powers = new Pose2d(vec.getX(),-vec.getY(),powers.getHeading());
//        robotRelative(powers);
//    }

    // Test this out again in teleOp (DriveTesting class)
    public void fieldRelative(Vector3D v) {
        Pose2d pose = new Pose2d(v.A, v.B, v.C);

        Vector2d vec = new Vector2d(pose.getX(),-pose.getY());

        vec = vec.rotated(angleWrap(getPoseEstimate().getHeading()));
        pose = new Pose2d(vec.getX(),-vec.getY(), -angleWrap(pose.getHeading()));

        robotRelative(pose);
    }

    public void goToPosePID(Vector3D input){
        odoDrive = -TranslationalPID_X.PIDF_Power(getPoseEstimate().getX(), input.A);
        odoStrafe = -TranslationalPID_Y.PIDF_Power(getPoseEstimate().getY(), input.B);
        odoTurn = -HeadingPID.PIDF_Power(angleWrap(getPoseEstimate().getHeading()), input.C);

        odoPID_Vector.set(odoDrive, odoStrafe, odoTurn);
        // Vector fields vector sum
        // Add odo vector and vector fields resultant vector

        setDriveVectorsFieldCentric(odoPID_Vector);
//        fieldRelative(PID_Vector);
    }

    public void goToPoseTrapezoidal(Vector3D input){
        odoDrive = -TranslationalProfile_X.getProfilePower(getPoseEstimate().getX(), input.A);
        odoStrafe = -TranslationalProfile_Y.getProfilePower(getPoseEstimate().getY(), input.B);
        odoTurn = -HeadingPID.PIDF_Power(angleWrap(getPoseEstimate().getHeading()), input.C);

        odoPID_Vector.set(odoDrive, odoStrafe, odoTurn);
        // Vector fields vector sum
        // Add odo vector and vector fields resultant vector

        trapezoidalTranslationalError = input.getVector2D().findDistance(getPoseVector().getVector2D());
        setDriveVectorsFieldCentric(odoPID_Vector);
//        fieldRelative(PID_Vector);
    }
    public void vectorFieldSum(){
        
    }
//    public void goToPose(Vector3D input){
//        inputArray = new Vector3D[]{odoCorrect(input), visionCorrect(input)};
//
//        for (Vector3D Input : inputArray){
//            PID_Vector.add(Input);
//        }
//        setDriveVectorsFieldCentric(PID_Vector);
//    }
//
//    public Vector3D odoCorrect(Vector3D input){
//        odoDrive = -TranslationalPID_X.PIDF_Power(getPoseEstimate().getX(), input.A);
//        odoStrafe = -TranslationalPID_Y.PIDF_Power(getPoseEstimate().getY(), input.B);
//        odoTurn = -HeadingPID.PIDF_Power(angleWrap(getPoseEstimate().getHeading()), input.C);
//
//        return new Vector3D(odoDrive, odoStrafe, odoTurn);
//    }
//
//    // TODO: Just use machine learning
    public void visionAlign(Vector2D input, PowerPlayPipeline pipeline){
        // conditions for each
        switch (trackingObject){
            case CONESTACK:
                switch (alliance){
                    case RED:
                        if (!pipeline.RedRect.empty()){
//                            visionDrive = visionX_PID.PIDF_Power(pipeline.RedRect.y + (pipeline.RedRect.height/2), input.A); //220
                            visionDrive = visionX_PID.PIDF_Power(pipeline.RedRect.width, input.A); //220
                            visionTurn = -visionHeading_PID.PIDF_Power(pipeline.getRectX(pipeline.RedRect), input.B); //160
                        }
                        break;
                    case BLUE:
                        if (!pipeline.BlueRect.empty()){
                            visionDrive = visionX_PID.PIDF_Power(pipeline.BlueRect.width, input.A); //220
                            visionTurn = -visionHeading_PID.PIDF_Power(pipeline.getRectX(pipeline.BlueRect), input.B); //160
                        }
                        break;
                }
                break;

            case JUNCTION:
                if (!pipeline.YellowRect.empty()){
                    visionDrive = -visionX_PID.PIDF_Power(pipeline.YellowRect.width, input.A); //
                    visionTurn = -visionHeading_PID.PIDF_Power(pipeline.getRectX(pipeline.YellowRect), input.B);
                }
                break;
            case NONE:
                break;
        }

        VisionAlignPID_Vector.set(visionDrive, 0, visionTurn);
        setDriveVectorsRobotCentric(VisionAlignPID_Vector);
    }

    //TODO: Test out field-centric
    public void ManualDrive(){
        controllerInput.set(Math.pow(gamepad1.left_stick_y, 3), Math.pow(gamepad1.left_stick_x, 3), Math.pow(gamepad1.right_stick_x, 3));
//        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsRobotCentric(controllerInput);
    }

    public void CorrectedManualDrive(){
        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setCorrectedDriveVectorsRobotCentric(controllerInput);
    }

    public void PID_Drive(){

    }

    public void DPad_Drive(){
        if (gamepad1.dpad_up){
            odoDrive = 0.25;
        } else if (gamepad1.dpad_down){
            odoDrive = -0.25;
        } else odoDrive = 0;

        if (gamepad1.dpad_left){
            odoStrafe = 0.25;
        } else if (gamepad1.dpad_right){
            odoStrafe = -0.25;
        } else {
            odoStrafe = 0;
        }

        controllerInput.set(odoDrive, odoStrafe, gamepad1.right_stick_x);
//        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsFieldCentric(controllerInput);
    }

    public void fieldCentricTest(){
        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsFieldCentric(controllerInput);
    }

    public void chassisTelemetry(){

    }
}
