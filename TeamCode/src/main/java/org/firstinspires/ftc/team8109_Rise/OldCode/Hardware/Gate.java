package org.firstinspires.ftc.team8109_Rise.OldCode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {

    Servo gate;

    double openPosition = 0;
    double closedPosition = 0;

    public enum gateState {
        OPEN,
        CLOSED
    }

    public gateState GateState;

    public Gate(String name, HardwareMap hardwareMap, double openPosition){
        gate = hardwareMap.get(Servo.class, name); // name is "gate"
        this.openPosition = openPosition;

        GateState = gateState.CLOSED;
    }

    public Gate(String name, HardwareMap hardwareMap, double openPosition, double closedPosition){
        gate = hardwareMap.get(Servo.class, name); // name is "gate"
        this.openPosition = openPosition;
        this.closedPosition = closedPosition;

        GateState = gateState.CLOSED;
    }

    public Gate(String name, HardwareMap hardwareMap, double openPosition, double closedPosition, gateState initialState){
        gate = hardwareMap.get(Servo.class, name); // name is "gate"

        this.openPosition = openPosition;
        this.closedPosition = closedPosition;

        GateState = initialState;
    }

    public void toggleGate(){
        switch (GateState) {
            case OPEN:
                gate.setPosition(openPosition);
                break;
            case CLOSED:
                gate.setPosition(closedPosition);
                break;
        }
    }

    public void setGatePosition(double target){
        gate.setPosition(target);
    }
}
