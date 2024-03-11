package org.firstinspires.ftc.robotcontroller.Hardware.Arms.ArmExtensions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class ArmWrist {
    public Servo wristServo;

    public ArmWrist(String name, HardwareMap hardwareMap){
        wristServo = hardwareMap.get(Servo.class, name);
    }

    public double getPositionDegrees(){
        return wristServo.getPosition()*300;
    }

    public void setAngle(double angle){
        wristServo.setPosition(angle/(270));
    }

}
