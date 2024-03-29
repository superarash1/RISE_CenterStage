package org.firstinspires.ftc.team8109_Rise.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcontroller.Hardware.Sensors.InertialMeasurementUnit;

public class Tenacity_IMU extends InertialMeasurementUnit {

    public Tenacity_IMU(HardwareMap hardwareMap){
        //Double check
        super(AxesOrder.XYZ, hardwareMap);
    }
}
