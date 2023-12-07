package org.firstinspires.ftc.team8109_Rise.OldCode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Cage {
    public ServoImplEx cageSpin;

    public Cage (String name, HardwareMap hardwareMap){
        cageSpin = hardwareMap.get(ServoImplEx.class, name);
    }
}
