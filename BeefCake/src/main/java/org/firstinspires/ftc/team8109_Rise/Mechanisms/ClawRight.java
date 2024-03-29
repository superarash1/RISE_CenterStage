package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcontroller.Hardware.Intakes.ServoClaw;

public class ClawRight extends ServoClaw {
    Gamepad gamepad1;
    Telemetry telemetry;

    //TODO convert baack
    static double openPosition = 0;
    static double closedPosition = 30;

//    static double openPosition = 235;
//    static double closedPosition = 230;

    boolean toggle1 = true;
    boolean toggle2 = false;

    boolean lastToggleY = false;

    public ClawRight(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super("clawRight", openPosition, closedPosition, hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void toggleClaw(){
        setPosition();
        switch (clawState){
            case OPEN:
                if ((gamepad1.y != lastToggleY) && gamepad1.y){
                    clawState = ClawState.CLOSED;
                }
                break;
            case CLOSED:
                if ((gamepad1.y != lastToggleY) && gamepad1.y){
                    clawState = ClawState.OPEN;
                }
                break;
        }
        lastToggleY = gamepad1.y;
    }

    public void setTelemetry(){
        telemetry.addData("Claw State", clawState);
        telemetry.addData("Claw Angle", getPositionDegrees());
        telemetry.addData("Claw Position", clawServo.getPosition());
    }
}
