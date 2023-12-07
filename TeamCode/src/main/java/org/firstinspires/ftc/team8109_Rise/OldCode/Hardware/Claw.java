package org.firstinspires.ftc.team8109_Rise.OldCode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Claw {
    public ServoImplEx clawServo;

    public Claw(String name, HardwareMap hardwareMap){
        clawServo = hardwareMap.get(ServoImplEx.class, name);
    }

    public Claw(String name,double openPosition, double closedPosition, HardwareMap hardwareMap){
        clawServo = hardwareMap.get(ServoImplEx.class, name);
    }

    public double clawPosition(){

        return clawServo.getPosition();
    }

}
