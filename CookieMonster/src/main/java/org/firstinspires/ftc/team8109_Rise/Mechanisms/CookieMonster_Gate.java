package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcontroller.Hardware.Random.GateExtension;

public class CookieMonster_Gate extends GateExtension {
    Gamepad gamepad1;
    Telemetry telemetry;

    //TODO convert back
    static double openPosition = 180;
    static double closedPosition = 300;

    boolean toggle1 = true;
    boolean toggle2 = false;

    boolean lastToggleTriangle = false;

    public CookieMonster_Gate(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super("gate", openPosition, closedPosition, 300, hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        gateState = GateState.OPEN;
    }

    public void toggleGate(){
        switch (gateState){
            case OPEN:
                setAngle(openPosition);

//                if ((gamepad1.triangle != lastToggleTriangle) && gamepad1.triangle && toggle1){
//                    toggle1 = false;
//                    toggle2 = true;
//
//                    gateState = GateState.CLOSED;
//                }

                if (gamepad1.triangle != lastToggleTriangle && gamepad1.triangle){
                    gateState = GateState.CLOSED;
                }

                break;
            case CLOSED:
                setAngle(closedPosition);
                if (gamepad1.triangle != lastToggleTriangle && gamepad1.triangle){
                    gateState = GateState.OPEN;
                }
                break;
        }
        lastToggleTriangle = gamepad1.triangle;
    }
}
