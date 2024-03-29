package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Arms.ArmExtensions.ArmWrist;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TenacityWrist extends ArmWrist {

    Gamepad gamepad1;
    Telemetry telemetry;

    public enum WristPosition{
        INIT,
        INTAKE_POSITION,
        OUTTAKE_FIRST_LINE,
        OUTTAKE_SECOND_LINE,
        OUTTAKE_THIRD_LINE,

    }

    public WristPosition wristPosition;

    public TenacityWrist(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super("wrist", hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        wristPosition = WristPosition.INIT;
    }

    public void setWristPosition(){
        switch (wristPosition){
            case INIT:
                wristServo.setPosition(0.75);
                break;
            case INTAKE_POSITION:
                wristServo.setPosition(0.75);
                break;
            case OUTTAKE_FIRST_LINE:
                wristServo.setPosition(0.75);
                break;
            case OUTTAKE_SECOND_LINE:
                wristServo.setPosition(0.75);
                break;
            case OUTTAKE_THIRD_LINE:
                wristServo.setPosition(0.75);
                break;
        }
    }
}
