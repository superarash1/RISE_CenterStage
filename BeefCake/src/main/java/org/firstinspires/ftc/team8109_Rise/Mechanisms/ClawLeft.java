package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcontroller.Hardware.Intakes.ServoClaw;

public class ClawLeft extends ServoClaw {
    Gamepad gamepad1;
    Telemetry telemetry;

    //TODO convert baack
    static double openPosition = 300;
    static double closedPosition = 300-30;

    boolean toggle1 = true;
    boolean toggle2 = false;

    boolean lastToggleY = false;

    public ClawLeft(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super("clawLeft", openPosition, closedPosition, hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    //TODO: reduce open angle,
    public void toggleClaw(){
        setPosition();
        switch (clawState){
            case OPEN:
                setAngle(openPosition);

                if ((gamepad1.y != lastToggleY) && gamepad1.y && toggle1){
                    toggle1 = false;
                    toggle2 = true;

                    clawState = ClawState.CLOSED;
                }
                break;
            case CLOSED:
                setAngle(closedPosition);

                if ((gamepad1.y != lastToggleY) && gamepad1.y && toggle2){
                    toggle2 = false;
                    toggle1 = true;

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
