package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Intakes.ServoClaw;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TenacityClaw {
    public ClawLeft clawLeft;
    public ClawRight clawRight;

    enum ClawState {
        OPEN,
        CLOSED
    }
    public ClawState clawState;
    Gamepad gamepad1;
    boolean lastToggleY = false;
    boolean lastToggleLeftBumper = false;
    boolean lastToggleRightBumper = false;

    public TenacityClaw(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        clawLeft = new ClawLeft(gamepad1, telemetry, hardwareMap);
        clawRight = new ClawRight(gamepad1, telemetry, hardwareMap);

        this.gamepad1 = gamepad1;

        clawState = ClawState.OPEN;
    }

    public void toggleClaw(){
        clawLeft.setPosition();
        clawRight.setPosition();

        switch (clawState){
            case OPEN:
                if ((gamepad1.y != lastToggleY) && gamepad1.y){
                    clawLeft.clawState = ServoClaw.ClawState.CLOSED;
                    clawRight.clawState = ServoClaw.ClawState.CLOSED;
                }
                break;
            case CLOSED:
                if ((gamepad1.y != lastToggleY) && gamepad1.y){
                    clawLeft.clawState = ServoClaw.ClawState.OPEN;
                    clawRight.clawState = ServoClaw.ClawState.OPEN;
                }
                break;
        }
        lastToggleY = gamepad1.y;
    }

    public void toggleLeftClaw(){
        clawLeft.setPosition();

        switch (clawState){
            case OPEN:
                if ((gamepad1.left_bumper != lastToggleLeftBumper) && gamepad1.left_bumper){
                    clawLeft.clawState = ServoClaw.ClawState.CLOSED;
                }
                break;
            case CLOSED:
                if ((gamepad1.left_bumper != lastToggleLeftBumper) && gamepad1.left_bumper){
                    clawLeft.clawState = ServoClaw.ClawState.OPEN;
                }
                break;
        }
        lastToggleY = gamepad1.y;
    }

    public void toggleRightClaw(){
        clawRight.setPosition();

        switch (clawState){
            case OPEN:
                if ((gamepad1.right_bumper != lastToggleRightBumper) && gamepad1.right_bumper){
                    clawRight.clawState = ServoClaw.ClawState.CLOSED;
                }
                break;
            case CLOSED:
                if ((gamepad1.left_bumper != lastToggleLeftBumper) && gamepad1.left_bumper){
                    clawRight.clawState = ServoClaw.ClawState.OPEN;
                }
                break;
        }
        lastToggleY = gamepad1.y;
    }
    public void setClawOpen(){
        clawLeft.setAngle(ClawLeft.openPosition);
        clawRight.setAngle(ClawRight.openPosition);
    }

    public void setClawClosed(){
        clawLeft.setAngle(ClawLeft.closedPosition);
        clawRight.setAngle(ClawRight.closedPosition);
    }

    public static class ClawRight extends ServoClaw {
        Gamepad gamepad1;
        Telemetry telemetry;

        //TODO convert baack
        static double openPosition = 0;
        static double closedPosition = 30;

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
                    setAngle(openPosition);

                    if ((gamepad1.y != lastToggleY) && gamepad1.y){

                        clawState = ClawState.CLOSED;
                    }
                    break;
                case CLOSED:
                    setAngle(closedPosition);

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

    public static class ClawLeft extends ServoClaw {
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
}
