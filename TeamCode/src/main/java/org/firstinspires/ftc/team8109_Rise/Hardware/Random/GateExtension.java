package org.firstinspires.ftc.team8109_Rise.Hardware.Random;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class GateExtension {
    public Servo gate;

    public double openPosition;
    public double closedPosition;
    public double range;

    public enum GateState{
        OPEN,
        CLOSED,
    }
    public GateState gateState;
    public GateExtension(String name, double openPosition, double closedPosition, double range, HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, name);

        this.openPosition = openPosition;
        this.closedPosition = closedPosition;
        this.range = range;
        gateState = GateState.OPEN;
    }

    public double getPositionAngle(){
        return gate.getPosition()*range;
    }

    public void setAngle(double angle){
        gate.setPosition(angle/(range));
    }

    public void setPosition(){
        switch (gateState){
            case OPEN:
                setAngle(openPosition);
                break;

            case CLOSED:
                setAngle(closedPosition);
                break;
        }
    }
}
