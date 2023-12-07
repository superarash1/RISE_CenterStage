package org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.ActiveGeckoClaw;

public class GeckoClawTestControl {
    public ActiveGeckoClaw claw;

    boolean toggle1 = true;
    boolean toggle2 = false;

    Telemetry telemetry;

    boolean intakeToggle1 = true;
    boolean intakeToggle2 = false;


    boolean lastToggleX = true;
    boolean lastToggleA = true;

    public enum ClawState {
        CLOSED,
        OPEN
    }

    public enum INTAKE_State {
        PASSIVE_MODE,
        ACTIVE_MODE
    }

    Gamepad gamepad1;

    ClawState clawState;
    INTAKE_State intake_state;
    public GeckoClawTestControl(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry){
        claw = new ActiveGeckoClaw("gecko1", "gecko2", "rotate1", "rotate2", hardwareMap);
        intake_state = INTAKE_State.ACTIVE_MODE;
        clawState = ClawState.CLOSED;

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
    }

    public void Claw(){
        switch (intake_state){
            case PASSIVE_MODE:
                switch (clawState){
                    case OPEN:
                        claw.rotate1.setPosition(0.9);
                        claw.rotate2.setPosition(1);

                        if ((gamepad1.x != lastToggleX) && gamepad1.x && toggle1){
                            toggle1 = false;
                            toggle2 = true;

                            clawState = ClawState.CLOSED;
                        }

                        lastToggleX = gamepad1.x;

                        break;

                    case CLOSED:
                        if ((gamepad1.x != lastToggleX) && gamepad1.x && toggle2){
                            toggle2 = false;
                            toggle1 = true;

                            clawState = ClawState.OPEN;
                        }

                        lastToggleX = gamepad1.x;

                        claw.rotate1.setPosition(1);
                        claw.rotate2.setPosition(0.9);
                        break;
                }

                if ((gamepad1.a != lastToggleA) && gamepad1.a && intakeToggle1){
                    intakeToggle1 = false;
                    intakeToggle2 = true;

                    ;intake_state = INTAKE_State.ACTIVE_MODE;
                }

                lastToggleA = gamepad1.a;


                break;
            case ACTIVE_MODE:
                if (gamepad1.b){
                    claw.geckoSpin1.setPower(-1);
                    claw.geckoSpin2.setPower(1);
                } else {
                    claw.geckoSpin1.setPower(0);
                    claw.geckoSpin2.setPower(0);
                }

                claw.rotate1.setPosition(1);
                claw.rotate2.setPosition(0.9);
                
                claw.geckoSpin1.setPower(1);
                claw.geckoSpin2.setPower(-1);

                if ((gamepad1.a != lastToggleA) && gamepad1.a && intakeToggle1){
                    intakeToggle1 = false;
                    intakeToggle2 = true;

                    ;intake_state = INTAKE_State.PASSIVE_MODE;
                }

                lastToggleA = gamepad1.a;

                break;
        }
    }

    public void Telemetry(){
        telemetry.addData("Rotate1", claw.rotate1.getPosition());
        telemetry.addData("Rotate2", claw.rotate2.getPosition());
        telemetry.update();
    }
}
