package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Intakes.ServoClaw;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bartholomew_Claw extends ServoClaw {
    Gamepad gamepad1;
    Telemetry telemetry;

    static double openPosition = 60;
    static double closedPosition = 0;

    boolean lastToggleY = false;

    public Bartholomew_Claw(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super("claw", openPosition, closedPosition, hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    //TODO: reduce open angle,
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
