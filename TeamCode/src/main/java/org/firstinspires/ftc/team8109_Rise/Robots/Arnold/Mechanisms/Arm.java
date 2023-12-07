package org.firstinspires.ftc.team8109_Rise.Robots.Arnold.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Control.PIDF_Controller;
import org.firstinspires.ftc.team8109_Rise.Hardware.Arms.MotorArm;

public class Arm extends MotorArm {
    Gamepad gamepad1;
    Telemetry telemetry;
    static String[] names = {"arm"};

    double ArmPower = 0;
    double targetAngle;
    public double cagePosition = 0;

    boolean lastToggleX = false;
    boolean lastToggle_DPadUp = false;
    boolean lastToggle_DPadDown = false;
    boolean lastToggle_RightBumper = false;

    public Servo Claw;

    public enum ArmState {
        HOME,
        DROP_OFF,
        PICK_UP,
        MANUAL
    }

    public PIDF_Controller ArmPID;
    ArmState armState;
    ArmState originalState;
    public Arm(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super(1, names, 751.8, 1, 0, -38, hardwareMap);

        Claw = hardwareMap.get(Servo.class, "cage");

//        motors[0].setDirectionReverse();

        //0.0001
        ArmPID = new PIDF_Controller(0, 0, 0, 0);

        targetAngle = -33.7;

        armState = ArmState.HOME;

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void SetArmPower(){
        ArmPower = ArmPID.PIDF_Power(getAngleDegrees(), targetAngle);
        switch (armState){
            case HOME:
                targetAngle = -33.7;
                cagePosition = 0.95;
                break;

            case PICK_UP:
                targetAngle = -33.7;
                cagePosition = 0.95;
                break;
            case DROP_OFF:
                targetAngle = -33.7;
                cagePosition = 0.95;
                break;

            case MANUAL:
                if (gamepad1.dpad_up){
                    ArmPower = -0.25;
                } else if (gamepad1.dpad_down){
                    ArmPower = 0.25;
                } else {
                    ArmPower = 0;
                }

                if (gamepad1.dpad_right){
                    cagePosition += 0.01;
                } else if (gamepad1.dpad_left){
                    cagePosition -= 0.01;
                }
                break;
        }
        setPower(ArmPower);
        Claw.setPosition(cagePosition);
    }

    public void DriverControl(){
        SetArmPower();
        switch (armState){
            case HOME:

                break;

            case DROP_OFF:
                if ((gamepad1.x != lastToggleX) && gamepad1.x){
                    armState = ArmState.PICK_UP;
                }
                break;

            case PICK_UP:
                if ((gamepad1.x != lastToggleX) && gamepad1.x){
                    armState = ArmState.DROP_OFF;
                }
                break;
            case MANUAL:
                if (gamepad1.right_bumper != lastToggle_RightBumper && gamepad1.right_bumper){
                    armState = originalState;
                }
                if (gamepad1.x != lastToggleX && gamepad1.x){
                    armState = ArmState.PICK_UP;
                }
                break;
        }

        lastToggleX = gamepad1.x;
    }

    public void Telemetry(){
        telemetry.addData("ArmPos Degrees", getAngleDegrees());
        telemetry.addData("Arm error", ArmPID.error);
        telemetry.addData("Arm target angle", targetAngle);
        telemetry.addData("ArmState", armState);
        telemetry.addData("ArmPower", ArmPower);
    }

    public void TuningTelemetry(){
        telemetry.addData("ArmPos Degrees", getAngleDegrees());
        telemetry.addData("Arm error", ArmPID.error);
        telemetry.addData("Arm target angle", targetAngle);
        telemetry.addData("ArmState", armState);
        telemetry.addData("ArmPower", ArmPower);
        telemetry.addData("Anti-Gravity", antiGravity);
        telemetry.addData("Proportion", ArmPID.P);
        telemetry.addData("Integral", ArmPID.I);
        telemetry.addData("Derivative", ArmPID.D);
    }
}
