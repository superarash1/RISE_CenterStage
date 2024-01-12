package org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SausageFingers {
    ClawLeft clawLeft;
    ClawRight clawRight;

    enum ClawState {
        OPEN,
        CLOSED
    }
    ClawState clawState;
    Gamepad gamepad1;
    public SausageFingers(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        clawLeft = new ClawLeft(gamepad1, telemetry, hardwareMap);
        clawRight = new ClawRight(gamepad1, telemetry, hardwareMap);

        this.gamepad1 = gamepad1;

        clawState = ClawState.OPEN;
    }

    public void toggleClaw(){
        clawLeft.toggleClaw();
        clawRight.toggleClaw();
    }

    public void setClawOpen(){
        clawLeft.setAngle(ClawLeft.openPosition);
        clawRight.setAngle(ClawRight.openPosition);
    }

    public void setClawClosed(){
        clawLeft.setAngle(ClawLeft.closedPosition);
        clawRight.setAngle(ClawRight.closedPosition);
    }
}
