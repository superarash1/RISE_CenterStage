package org.firstinspires.ftc.team8109_Rise.Sensors.Odometry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Resources.RoadRunnerQuickstart.util.Encoder;
import org.firstinspires.ftc.robotcontroller.Hardware.Sensors.Odometry.StandardTwoWheelOdoLocalizer;
import org.firstinspires.ftc.team8109_Rise.Sensors.Tenacity_IMU;

public class Tenacity_OdometryLocalizer extends StandardTwoWheelOdoLocalizer {

    static String[] configNames = {"fRight", "fLeft"};

    static double X_MULTIPLIER = 1.013662406346408; // Multiplier in the X direction  1.011395
    static double Y_MULTIPLIER = 1.01596837246284; // Multiplier in the Y direction  1.0125

    public static double PARALLEL_X = 0; // X is the forward and back direction
    public static double PARALLEL_Y = 0; // Y is the strafe direction

    public static double PERPENDICULAR_X = 0; // X is the forward and back direction
    public static double PERPENDICULAR_Y = 0; // Y is the strafe direction
    static double[] OdoConstants = {PARALLEL_X, PARALLEL_Y, PERPENDICULAR_X, PERPENDICULAR_Y};

    public Tenacity_OdometryLocalizer(HardwareMap hardwareMap) {
        super(new TenacityDriveConstants(), OdoConstants, new Tenacity_IMU(hardwareMap), configNames, X_MULTIPLIER, Y_MULTIPLIER, hardwareMap);
        // TODO: test if any encoders need to be reversed
        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
    }
}