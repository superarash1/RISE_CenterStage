package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team8109_Rise.Hardware.Arms.ArmExtensions.ArmWrist;

public class Wrist extends ArmWrist {

    Gamepad gamepad1;

    public enum WristPosition{
        INTAKE_POSITION,
        OUTTAKE_POSITION
    }

    public WristPosition wristPosition;

    boolean toggle1 = true;
    boolean toggle2 = false;

    boolean lastToggleX = false;

    public Wrist(Gamepad gamepad1, HardwareMap hardwareMap) {
        super("wrist", hardwareMap);

        this.gamepad1 = gamepad1;
        wristPosition = WristPosition.INTAKE_POSITION;
    }

    public void setPosition(){
        switch (wristPosition){
            case INTAKE_POSITION:
                setAngle(237.5);
                if ((gamepad1.x != lastToggleX) && gamepad1.x && toggle1){
                    toggle1 = false;
                    toggle2 = true;

                    wristPosition = WristPosition.OUTTAKE_POSITION;
                }
                break;

            case OUTTAKE_POSITION:
                setAngle(57.5);

                if ((gamepad1.x != lastToggleX) && gamepad1.x && toggle2){
                    toggle2 = false;
                    toggle1 = true;

                    wristPosition = WristPosition.INTAKE_POSITION;
                }
                break;
        }

        lastToggleX = gamepad1.x;
    }
}
