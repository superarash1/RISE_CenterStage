package org.firstinspires.ftc.robotcontroller.Hardware.Odometry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class ServoOdoRetract {
    public Servo retractionServo[];
    public double[] groundPosition;
    public double[] retractPosition;
    public int podCount;

    public ServoOdoRetract (int podCount, double[] groundPosition, double[] retractPosition, String[] name, HardwareMap hardwareMap){
        retractionServo = new Servo[podCount];
        this.groundPosition = groundPosition;
        this.retractPosition = retractPosition;

        for (int i = 0; i < podCount; i++) {
            retractionServo[i] = hardwareMap.get(Servo.class, name[i]);
        }

        this.podCount = podCount;
    }
}