package org.firstinspires.ftc.robotcontroller.Hardware.Lifts;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Motor;

public abstract class Slides {

    public Motor motors[];

    public double pulleyRadius;
    public double kGravity = 0;

    public double height;
    public double stages;

    public enum StringingMethod {
        CONTINUOUS,
        CASCADE
    }

    public StringingMethod method;

    public Slides(int motorCount, String[] name, double pulleyRadius, StringingMethod method, int stages, double kGravity, HardwareMap hardwareMap){
        Motor motors[] = new Motor[motorCount];

        for (int i = 0; i <motors.length; i++){
            motors[i] = new Motor(name[i], 384.5, hardwareMap);
        }

        this.pulleyRadius = pulleyRadius;

        this.method = method;
        this.stages = stages;
        this.motors = motors;
        this.kGravity = kGravity;
    }

    public void setPower(double power){
        for (Motor motor:motors) {
            motor.setPower(kGravity + power);
        }
    }

    public double getExtension(){
        if (method == StringingMethod.CONTINUOUS){
            height = Math.toRadians(motors[0].getCurrPosDegrees()) * pulleyRadius;
        } else if (method == StringingMethod.CASCADE){
            height = stages*(motors[0].getCurrPosRadians() * pulleyRadius);
        }

        return height;
    }
}
