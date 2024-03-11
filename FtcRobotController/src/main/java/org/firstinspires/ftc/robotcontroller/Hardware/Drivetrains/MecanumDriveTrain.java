package org.firstinspires.ftc.robotcontroller.Hardware.Drivetrains;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcontroller.Control.MotionProfiling.TrapezoidalMotionProfile;
import org.firstinspires.ftc.robotcontroller.Control.PID_Controller;
import org.firstinspires.ftc.robotcontroller.Hardware.Motor;
import org.firstinspires.ftc.robotcontroller.Hardware.Resources.RoadRunnerQuickstart.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.robotcontroller.Hardware.Resources.RoadRunnerQuickstart.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.robotcontroller.Hardware.Resources.RoadRunnerQuickstart.util.LynxModuleUtil;
import org.firstinspires.ftc.robotcontroller.Hardware.Sensors.DriveConstants_Mecanum;
import org.firstinspires.ftc.robotcontroller.Hardware.Sensors.InertialMeasurementUnit;
import org.firstinspires.ftc.robotcontroller.Hardware.Sensors.Odometry.StandardThreeWheelOdoLocalizer;
import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public abstract class MecanumDriveTrain extends MecanumDrive {

    public Motor frontLeft;
    public Motor frontRight;
    public Motor backRight;
    public Motor backLeft;

    public Pose2d poseEstimate;

//    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
//    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    double VX_WEIGHT;
    double VY_WEIGHT;
    double OMEGA_WEIGHT;

    public double fLeft;
    public double fRight;
    public double bLeft;
    public double bRight;

    public double odoDrive;
    public double odoStrafe;
    public double odoTurn;

    public double max;

    public double x_rotated;
    public double y_rotated;

    public enum OdometryType{
        THREE_WHEEL_ODO,
        TWO_WHEEL_ODO,
        DRIVE_ENCODERS
    }
    OdometryType odometryType;

    public double previousFLeft = 0;
    public double previousFRight = 0;
    public double previousBRight = 0;
    public double previousBLeft = 0;

    private static TrajectoryVelocityConstraint VEL_CONSTRAINT;
    private static TrajectoryAccelerationConstraint ACCEL_CONSTRAINT;

    public Vector3D controllerInput = new Vector3D(0, 0, 0);

    public PID_Controller TranslationalPID_X;
    public PID_Controller TranslationalPID_Y;
    public PID_Controller HeadingPID;

    public TrapezoidalMotionProfile TranslationalProfile_X;
    public TrapezoidalMotionProfile TranslationalProfile_Y;

    public double trapezoidalTranslationalError = 0;

    public Vector3D odoPID_Vector = new Vector3D(0, 0, 0);

    public Vector3D RobotPose;

    private final TrajectorySequenceRunner trajectorySequenceRunner;

    private TrajectoryFollower follower;

    private List<Motor> motors;


    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public org.firstinspires.ftc.robotcontroller.Hardware.Sensors.InertialMeasurementUnit InertialMeasurementUnit;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    public Gamepad gamepad1;
    public Telemetry telemetry;

    public DriveConstants_Mecanum CONSTANTS;

    public double rpmToVelocity(double rpm) {
        return rpm * CONSTANTS.GEAR_RATIO * 2 * Math.PI * CONSTANTS.WHEEL_RADIUS / 60.0;
    }

    public double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }

    //TODO: take out all thig of other drive constant files (parameter to input a DriveConstants class object?)
    public MecanumDriveTrain(String flName, String frName, String brName, String blName, DriveConstants_Mecanum CONSTANTS,
                             OdometryType odometryType, StandardThreeWheelOdoLocalizer localizer, HardwareMap hardwareMap) {

        //TODO: Make it different files
        super(CONSTANTS.kV, CONSTANTS.kA, CONSTANTS.kStatic, CONSTANTS.TRACK_WIDTH, CONSTANTS.WHEEL_BASE, CONSTANTS.LATERAL_MULTIPLIER);

        this.CONSTANTS = CONSTANTS;
        VEL_CONSTRAINT = getVelocityConstraint(CONSTANTS.MAX_VEL, CONSTANTS.MAX_ANG_VEL, CONSTANTS.TRACK_WIDTH);

        ACCEL_CONSTRAINT = getAccelerationConstraint(CONSTANTS.MAX_ACCEL);

        if (odometryType == OdometryType.DRIVE_ENCODERS) InertialMeasurementUnit = new InertialMeasurementUnit(hardwareMap);

        follower = new HolonomicPIDVAFollower(CONSTANTS.TRANSLATIONAL_PID, CONSTANTS.TRANSLATIONAL_PID, CONSTANTS.HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        frontLeft = new Motor(flName, hardwareMap);
        frontRight = new Motor(frName, hardwareMap);
        backRight = new Motor(brName, hardwareMap);
        backLeft = new Motor(blName, hardwareMap);

        frontRight.setDirectionReverse();
        backRight.setDirectionReverse();
        frontLeft.setDirectionForward();
        backLeft.setDirectionForward();

        RobotPose = new Vector3D(getPoseEstimate().getX(), getPoseEstimate().getY(), getPoseEstimate().getHeading());

        motors = Arrays.asList(frontLeft, frontRight, backRight, backLeft);

        for (Motor motor : motors) {
            motor.setBreakMode();
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (CONSTANTS.RUN_USING_ENCODER && CONSTANTS.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, CONSTANTS.MOTOR_VELO_PID);
        }

        // TODO: if desired, use setLocalizer() to change the localization method
        if (odometryType == OdometryType.THREE_WHEEL_ODO || odometryType == OdometryType.TWO_WHEEL_ODO){
            setLocalizer(localizer);
        }

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }


    public MecanumDriveTrain(String flName, String frName, String brName, String blName, DriveConstants_Mecanum CONSTANTS,
                             OdometryType odometryType, HardwareMap hardwareMap) {

        //TODO: Make it different files
        super(CONSTANTS.kV, CONSTANTS.kA, CONSTANTS.kStatic, CONSTANTS.TRACK_WIDTH, CONSTANTS.WHEEL_BASE, CONSTANTS.LATERAL_MULTIPLIER);

        this.CONSTANTS = CONSTANTS;
        VEL_CONSTRAINT = getVelocityConstraint(CONSTANTS.MAX_VEL, CONSTANTS.MAX_ANG_VEL, CONSTANTS.TRACK_WIDTH);

        ACCEL_CONSTRAINT = getAccelerationConstraint(CONSTANTS.MAX_ACCEL);

        if (odometryType == OdometryType.DRIVE_ENCODERS) InertialMeasurementUnit = new InertialMeasurementUnit(hardwareMap);

        follower = new HolonomicPIDVAFollower(CONSTANTS.TRANSLATIONAL_PID, CONSTANTS.TRANSLATIONAL_PID, CONSTANTS.HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        frontLeft = new Motor(flName, hardwareMap);
        frontRight = new Motor(frName, hardwareMap);
        backRight = new Motor(brName, hardwareMap);
        backLeft = new Motor(blName, hardwareMap);

        frontRight.setDirectionReverse();
        backRight.setDirectionReverse();
        frontLeft.setDirectionForward();
        backLeft.setDirectionForward();

        RobotPose = new Vector3D(getPoseEstimate().getX(), getPoseEstimate().getY(), getPoseEstimate().getHeading());

        motors = Arrays.asList(frontLeft, frontRight, backRight, backLeft);

        for (Motor motor : motors) {
            motor.setBreakMode();
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (CONSTANTS.RUN_USING_ENCODER && CONSTANTS.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, CONSTANTS.MOTOR_VELO_PID);
        }

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

    public void setDriveVectorsRobotCentric(Vector3D input){
        fLeft = CONSTANTS.VX_WEIGHT * input.A - CONSTANTS.VY_WEIGHT * input.B - CONSTANTS.OMEGA_WEIGHT * input.C;
        fRight = CONSTANTS.VX_WEIGHT * input.A + CONSTANTS.VY_WEIGHT * input.B + CONSTANTS.OMEGA_WEIGHT * input.C;
        bRight = CONSTANTS.VX_WEIGHT * input.A - CONSTANTS.VY_WEIGHT * input.B + CONSTANTS.OMEGA_WEIGHT * input.C;
        bLeft = CONSTANTS.VX_WEIGHT * input.A + CONSTANTS.VY_WEIGHT * input.B - CONSTANTS.OMEGA_WEIGHT * input.C;

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

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                CONSTANTS.MAX_ANG_VEL, CONSTANTS.MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public double angleWrap(double radians) {
        if (radians > Math.PI) {
            radians -= 2*Math.PI;
        }

        if (radians < -Math.PI) {
            radians = radians - 2*Math.PI;
        }

        return radians;
    }

    public Vector3D getPoseVector(){
        return new Vector3D(getPoseEstimate().getX(), getPoseEstimate().getY(), angleWrap(getPoseEstimate().getHeading()));
    }
//
//    public Pose2d PoseEstimate(){
//        poseEstimate = getPoseEstimate();
//        return poseEstimate;
//    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (Motor motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (Motor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (Motor motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (Motor motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrPosTicks()));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (Motor motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        frontLeft.setPower(v);
        backLeft.setPower(v1);
        backRight.setPower(v2);
        frontRight.setPower(v3);
    }

    public double getRawExternalHeading() {
        return 0;
    }

    public double encoderTicksToInches(double ticks) {
        return CONSTANTS.WHEEL_RADIUS * 2 * Math.PI * CONSTANTS.GEAR_RATIO * ticks / CONSTANTS.TICKS_PER_REV;
    }

    public Double getExternalHeadingVelocity() {
        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
        // expected). This bug does NOT affect orientation.
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) -InertialMeasurementUnit.imu.getAngularVelocity().xRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void setPower(double power){
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backRight.setPower(power);
        this.backLeft.setPower(power);
    }

    public void setPower(double fLeft, double fRight, double bRight, double bLeft){
        this.frontLeft.setPower(fLeft);
        this.frontRight.setPower(fRight);
        this.backRight.setPower(bRight);
        this.backLeft.setPower(bLeft);
    }

    public void setVelocity(double ω){
        this.frontLeft.setVelocity(ω);
        this.frontRight.setVelocity(ω);
        this.backRight.setVelocity(ω);
        this.backLeft.setVelocity(ω);
    }

    public void setVelocity(double fLeft, double fRight, double bRight, double bLeft){
        this.frontLeft.setVelocity(fLeft);
        this.frontRight.setVelocity(fRight);
        this.backRight.setVelocity(bRight);
        this.backLeft.setVelocity(bLeft);
    }

    public void reset(){
        this.frontLeft.reset();
        this.frontRight.reset();
        this.backRight.reset();
        this.backLeft.reset();
    }

    public void setBreakMode(){
        this.frontLeft.setBreakMode();
        this.frontRight.setBreakMode();
        this.backRight.setBreakMode();
        this.backLeft.setBreakMode();
    }

    public void setFloatMode(){
        this.frontLeft.setFloatMode();
        this.frontRight.setFloatMode();
        this.backRight.setFloatMode();
        this.backLeft.setFloatMode();
    }
}
