package org.firstinspires.ftc.robotcontroller.Hardware.Sensors.Odometry;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Resources.RoadRunnerQuickstart.util.Encoder;
import org.firstinspires.ftc.robotcontroller.Hardware.Sensors.DriveConstants_Mecanum;
import org.firstinspires.ftc.robotcontroller.Hardware.Sensors.InertialMeasurementUnit;

import java.util.Arrays;
import java.util.List;

public abstract class StandardTwoWheelOdoLocalizer extends TwoTrackingWheelLocalizer {
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

    public Encoder parallelEncoder, perpendicularEncoder;

    public InertialMeasurementUnit imu;

    //TODO: Figure out the whole IMU situation and then add in the rest of 2 wheel odo code as instructed on learn roadrunner

    public StandardTwoWheelOdoLocalizer(DriveConstants_Mecanum driveConstants, double[] odoConstants, InertialMeasurementUnit imu, String[] configNames, double X_MULTIPLIER, double Y_MULTIPLIER, HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(odoConstants[0], odoConstants[1] / 2, 0), // Parallel Wheel
                new Pose2d(odoConstants[2], -odoConstants[3] / 2, 0) // Perpendicular Wheel
        ));

        // TODO: adjust the names of the following hardware devices to match your configuration

        TICKS_PER_REV = driveConstants.TICKS_PER_REV;
        WHEEL_RADIUS = driveConstants.WHEEL_RADIUS; // in
        GEAR_RATIO = driveConstants.GEAR_RATIO; // output (wheel) speed / input (encoder) speed

        LATERAL_DISTANCE = driveConstants.LATERAL_DISTANCE; // in; distance between the left and right wheels
        FORWARD_OFFSET = driveConstants.FORWARD_OFFSET; // in; offset of the lateral wheel

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, configNames[1]));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, configNames[2]));

        this.imu = imu;

        this.X_MULTIPLIER = X_MULTIPLIER;
        this.Y_MULTIPLIER = Y_MULTIPLIER;
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()),
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())
        );
    }

    public double getRawExternalHeading() {
        return imu.getFirstAngle();
    }

    public Double getExternalHeadingVelocity() {
        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
        // expected). This bug does NOT affect orientation.
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) -imu.getXAngularVelocity();
    }
}
