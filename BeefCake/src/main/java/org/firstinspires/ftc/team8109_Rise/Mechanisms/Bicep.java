package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Arms.ServoArm;

public class Bicep extends ServoArm {


    public enum BicepStates{
        HOME,
        FIRST_LINE,
        THIRD_LINE,
        MANUAL
    }

    public BicepStates bicepState;
    public Bicep(String[] name, HardwareMap hardwareMap) {
        super(ServoArmType.DOUBLE_SERVO, name, hardwareMap);

        bicepState = BicepStates.HOME;
    }

    public void bicepStates(){
        switch (bicepState){
            case HOME:
                setAngle(0);
                break;
            case FIRST_LINE:
                setAngle(270);
                break;

            case THIRD_LINE:
                setAngle(270);
                break;

            case MANUAL:

                break;
        }
    }
}