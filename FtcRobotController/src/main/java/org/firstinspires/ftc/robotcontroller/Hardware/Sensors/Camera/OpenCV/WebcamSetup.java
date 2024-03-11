package org.firstinspires.ftc.robotcontroller.Hardware.Sensors.Camera.OpenCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;

public class WebcamSetup {
    OpenCvCamera webcam;

    Telemetry telemetry;
    HardwareMap hardwareMap;
    public WebcamSetup(OpenCvCamera webcam, Telemetry telemetry, HardwareMap hardwareMap){
        this.webcam = webcam;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void cameraSetup(){

    }
}
