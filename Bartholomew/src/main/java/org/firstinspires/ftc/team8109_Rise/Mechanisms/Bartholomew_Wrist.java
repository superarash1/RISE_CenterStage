package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Arms.ArmExtensions.ArmWrist;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bartholomew_Wrist extends ArmWrist {

    Gamepad gamepad1;
    Telemetry telemetry;

    public enum WristPosition{
        INTAKE_POSITION,
        OUTTAKE_POSITION,
        MANUAL
    }

    public WristPosition wristPosition;

    public Bartholomew_Wrist(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super("wrist", hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        wristPosition = WristPosition.INTAKE_POSITION;
    }

    public void setPosition(){
        switch (wristPosition){
            case INTAKE_POSITION:
                setAngle(237.5);
                break;

            case OUTTAKE_POSITION:
                setAngle(57.5);
                break;
            case MANUAL:
                if (gamepad1.left_trigger > 0.2){
                    setAngle(getPositionDegrees()-1);
                } else if (gamepad1.right_trigger > 0.2) {
                    setAngle(getPositionDegrees()+1);
                }
                break;
        }
    }
}
