package org.firstinspires.ftc.robotcontroller.Hardware.Intakes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class ServoClaw {
    public Servo clawServo;

    public double openPosition;
    public double closedPosition;

    public enum ClawState{
        OPEN,
        CLOSED
    }

    public ClawState clawState;

    public ServoClaw(String name, double openPosition, double closedPosition, HardwareMap hardwareMap){
        clawServo = hardwareMap.get(Servo.class, name);

        this.openPosition = openPosition;
        this.closedPosition = closedPosition;
        clawState = ClawState.OPEN;
    }

    public double getPositionDegrees(){
        return clawServo.getPosition()*300;
    }

    public void setAngle(double angle){
        clawServo.setPosition(angle/(300));
    }

    public void setPosition(){
        switch (clawState){
            case OPEN:
                setAngle(openPosition);
                break;

            case CLOSED:
                setAngle(closedPosition);
                break;
        }
    }

    public void setPositionTest(){
        switch (clawState){
            case OPEN:
                setAngle(270);
                break;

            case CLOSED:
                setAngle(250);
                break;
        }
    }
}
