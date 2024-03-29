package org.firstinspires.ftc.robotcontroller.Hardware.Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class InertialMeasurementUnit {

    public BNO055IMU imu;

    public Orientation straight;
    public double globalAngle;
    public InertialMeasurementUnit(HardwareMap hardwareMap){
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // IMU Setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
            /*
            We never use the accelerometer functions of the imu, so we set the
            accel unit to milli-earth-gravities as a joke
            */

        imu.initialize(parameters);

        straight = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.RADIANS);
    }
    public InertialMeasurementUnit(AxesOrder order, HardwareMap hardwareMap){
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // IMU Setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
            /*
            We never use the accelerometer functions of the imu, so we set the
            accel unit to milli-earth-gravities as a joke
            */

        imu.initialize(parameters);

        straight = imu.getAngularOrientation(AxesReference.INTRINSIC, order, AngleUnit.RADIANS);
    }

    public InertialMeasurementUnit(AxesOrder order, AngleUnit unit, HardwareMap hardwareMap){
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // IMU Setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
            /*
            We never use the accelerometer functions of the imu, so we set the
            accel unit to milli-earth-gravities as a joke
            */

        imu.initialize(parameters);

        straight = imu.getAngularOrientation(AxesReference.INTRINSIC, order, AngleUnit.RADIANS);
    }

    public double Angle_FieldCentric(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.RADIANS);
        globalAngle = angles.firstAngle - straight.firstAngle;

        return -globalAngle;
    }

    public double getFirstAngle(){
        return imu.getAngularOrientation().firstAngle;
    }

    public double getSecondAngle(){
        return imu.getAngularOrientation().firstAngle;
    }

    public double getThirdAngle(){
        return imu.getAngularOrientation().firstAngle;
    }

    public double getXAngularVelocity(){
        return imu.getAngularVelocity().xRotationRate;
    }

    public double getYAngularVelocity(){
        return imu.getAngularVelocity().yRotationRate;
    }

    public double getZAngularVelocity(){
        return imu.getAngularVelocity().zRotationRate;
    }
}
