package org.firstinspires.ftc.team8109_Rise.OldCode.Hardware;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ActiveGeckoClaw {
    public CRServoImplEx geckoSpin1;
    public CRServoImplEx geckoSpin2;

    public ServoImplEx rotate1;
    public ServoImplEx rotate2;

    public ActiveGeckoClaw(String gecko1Name, String gecko2Name, String rotate1Name, String rotate2Name, HardwareMap hardwareMap){
        geckoSpin1 = hardwareMap.get(CRServoImplEx.class, gecko1Name);
        geckoSpin2 = hardwareMap.get(CRServoImplEx.class, gecko2Name);
        rotate1 = hardwareMap.get(ServoImplEx.class, rotate1Name);
        rotate2 = hardwareMap.get(ServoImplEx.class, rotate2Name);
    }
}
