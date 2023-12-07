package org.firstinspires.ftc.team8109_Rise.Sensors.Odometry;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team8109_Rise.Resources.RoadRunnerQuickstart.util.Encoder;

import java.util.Arrays;
import java.util.List;

public abstract class StandardThreeWheelOdoLocalizer extends ThreeTrackingWheelLocalizer {
    /*
     * Sample tracking wheel localizer implementation assuming the standard configuration:
     *
     *    /--------------\
     *    |     ____     |
     *    |     ----     |
     *    | ||        || |
     *    | ||        || |
     *    |              |
     *    |              |
     *    \--------------/
     *
     */
//
//    public static double TICKS_PER_REV = 0;
//    public static double WHEEL_RADIUS = 0; // in
//    public static double GEAR_RATIO = 0; // output (wheel) speed / input (encoder) speed
//
//    public static double LATERAL_DISTANCE = 0; // in; distance between the left and right wheels
//    public static double FORWARD_OFFSET = 0; // in; offset of the lateral wheel

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.688975; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10.915; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4.5; // in; offset of the lateral wheel

    public double X_MULTIPLIER; // Multiplier in the X direction 40
    public double Y_MULTIPLIER; // Multiplier in the Y direction

    public Encoder leftEncoder, rightEncoder, middleEncoder;

    public StandardThreeWheelOdoLocalizer(double[] DriveConstants, String[] configNames, double X_MULTIPLIER, double Y_MULTIPLIER, HardwareMap hardwareMap) {
        super(Arrays.asList(
//                new Pose2d(0, DriveConstants[3] / 2, 0), // left
//                new Pose2d(0, -DriveConstants[3] / 2, 0), // right
//                new Pose2d(DriveConstants[4], 0, Math.toRadians(90)) // front

                new Pose2d(0, DriveConstants[3] / 2, 0), // left
                new Pose2d(0, -DriveConstants[3] / 2, 0), // right
                new Pose2d(DriveConstants[4], -1.4, Math.toRadians(90)) // front
        ));

        TICKS_PER_REV = DriveConstants[0];
        WHEEL_RADIUS = DriveConstants[1]; // in
        GEAR_RATIO = DriveConstants[2]; // output (wheel) speed / input (encoder) speed

        LATERAL_DISTANCE = DriveConstants[3]; // in; distance between the left and right wheels
        FORWARD_OFFSET = DriveConstants[4]; // in; offset of the lateral wheel

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, configNames[0]));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, configNames[1]));
        middleEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, configNames[2]));

//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        this.X_MULTIPLIER = X_MULTIPLIER;
        this.Y_MULTIPLIER = Y_MULTIPLIER;

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(middleEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(middleEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
