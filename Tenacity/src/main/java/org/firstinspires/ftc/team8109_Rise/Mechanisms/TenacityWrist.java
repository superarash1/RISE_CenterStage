package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Arms.ArmExtensions.ArmWrist;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
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

    public static double wristPosIntake = 0.55;
    public static double wristPosOuttake1 = 0.5;
    public static double wristPosOuttake2 = 0.5;
    public static double wristPosOuttake3 = 0.5;

    public WristPosition wristPosition;

    public TenacityWrist(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super("wrist", hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        wristPosition = WristPosition.INIT;
    }

    public void setWristPosition(){
        switch (wristPosition){
            case INTAKE_POSITION:
                wristServo.setPosition(wristPosIntake);
                break;
            case OUTTAKE_FIRST_LINE:
                wristServo.setPosition(wristPosOuttake1);
                break;
            case OUTTAKE_SECOND_LINE:
                wristServo.setPosition(wristPosOuttake2);
                break;
            case OUTTAKE_THIRD_LINE:
                wristServo.setPosition(wristPosOuttake3);
                break;
        }
    }
}
