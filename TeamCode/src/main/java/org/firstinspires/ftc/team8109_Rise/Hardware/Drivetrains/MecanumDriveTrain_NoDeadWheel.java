package org.firstinspires.ftc.team8109_Rise.Hardware.Drivetrains;

import static org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Sensors.SlidesBot_DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Sensors.SlidesBot_DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Sensors.SlidesBot_DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Sensors.SlidesBot_DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Sensors.SlidesBot_DriveConstants.encoderTicksToInches;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.team8109_Rise.Hardware.Motor;
import org.firstinspires.ftc.team8109_Rise.Math.Vectors.Vector3D;
import org.firstinspires.ftc.team8109_Rise.OldCode.InertialMeasurementUnit;
import org.firstinspires.ftc.team8109_Rise.Resources.RoadRunnerQuickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team8109_Rise.Resources.RoadRunnerQuickstart.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.team8109_Rise.Resources.RoadRunnerQuickstart.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.team8109_Rise.Resources.RoadRunnerQuickstart.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public abstract class MecanumDriveTrain_NoDeadWheel extends MecanumDrive {
    //Declare the variables for the mecanum drive train class
    /* Although the encoders aren't DcMotors, they can be initialized as one
    since they are connected to the drive train motor encoder ports on the rev hub.
    For Example, if the Left Encoder is attached to the port associated with the tl
    Motor , then the hardware name for the Left Encoder and the tl Motor would be the same
    */

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

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private TrajectoryVelocityConstraint VEL_CONSTRAINT;
    private TrajectoryAccelerationConstraint ACCEL_CONSTRAINT;

    private TrajectoryFollower follower;

    private List<Motor> motors;

    public InertialMeasurementUnit InertialMeasurementUnit;
    private VoltageSensor batteryVoltageSensor;


    //TODO: take out all thig of other drive constant files (parameter to input a DriveConstants class object?)
    public MecanumDriveTrain_NoDeadWheel(String flName, String frName, String brName, String blName,
                                         double kV, double kA, double kStatic,
                                         double TRACK_WIDTH, double WHEEL_BASE, double LATERAL_MULTIPLIER,
                                         PIDCoefficients TRANSLATIONAL_PID, PIDCoefficients HEADING_PID,
                                         double VX_WEIGHT, double VY_WEIGHT, double OMEGA_WEIGHT,
                                         TrajectoryVelocityConstraint VEL_CONSTRAINT, TrajectoryAccelerationConstraint ACCEL_CONSTRAINT,
                                         HardwareMap hardwareMap) {

        //TODO: Make it different files
        super(kV, kA, kStatic, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);

        //TODO: make imu usage for drive encoders easily toggleable
//        InertialMeasurementUnit = new InertialMeasurementUnit(hardwareMap);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
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

        motors = Arrays.asList(frontLeft, frontRight, backRight, backLeft);

        for (Motor motor : motors) {
            motor.setBreakMode();
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: if desired, use setLocalizer() to change the localization method
//        setLocalizer(new OdometryLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

        this.VEL_CONSTRAINT = VEL_CONSTRAINT;
        this.ACCEL_CONSTRAINT = ACCEL_CONSTRAINT;

        this.VX_WEIGHT = VX_WEIGHT;
        this.VY_WEIGHT = VY_WEIGHT;
        this.OMEGA_WEIGHT = OMEGA_WEIGHT;
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
                MAX_ANG_VEL, MAX_ANG_ACCEL
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

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
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
