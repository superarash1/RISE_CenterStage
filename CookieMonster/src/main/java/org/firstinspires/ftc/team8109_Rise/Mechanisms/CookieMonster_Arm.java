package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcontroller.Control.PID_Controller;
import org.firstinspires.ftc.robotcontroller.Hardware.Arms.MotorArm;

public class CookieMonster_Arm extends MotorArm {
    Gamepad gamepad1;
    Telemetry telemetry;
    static String[] names = {"leftArm", "rightArm"};

    double ArmPower = 0;
    double targetAngle;
    public double cagePosition = 0;

    boolean lastToggleX = false;
    boolean lastToggle_DPadUp = false;
    boolean lastToggle_DPadDown = false;
    boolean lastToggle_RightBumper = false;

    public Servo Cage;

    public enum ArmState {
        HOME,
        HIGH_LEVEL,
        MEDIUM_LEVEL,
        LOW_LEVEL,
        MANUAL
    }

    public PID_Controller ArmPID;

    ArmState armState;
    ArmState originalState;
    public CookieMonster_Arm(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super(2, names, 537.7, 0.2380952380952381, 0.035, -38, hardwareMap);

        Cage = hardwareMap.get(Servo.class, "cage");

        motors[0].setDirectionReverse();
        motors[1].setDirectionReverse();

        //0.0001
        ArmPID = new PID_Controller(0.0065, 0.0006, 0, 0.00045);

        targetAngle = -33.7;

        armState = ArmState.HOME;

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void SetArmPower(){
        ArmPower = ArmPID.PID_Power(getAngleDegrees(), targetAngle);
        switch (armState){
            case HOME:
                targetAngle = -33.7;
                cagePosition = 0.95;
                break;

            case HIGH_LEVEL:
                targetAngle = 114;
                if (getAngle() < 45) cagePosition = 0.25;
                break;

            case MEDIUM_LEVEL:
                targetAngle = 190;
                cagePosition = 0.2;
                break;

            case LOW_LEVEL:
                targetAngle = 230;
                cagePosition = 0.2;
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
        Cage.setPosition(cagePosition);
    }

    public void DriverControl(){
        SetArmPower();
        switch (armState){
            case HOME:
                if (gamepad1.right_bumper != lastToggle_RightBumper && gamepad1.right_bumper){
                    armState = ArmState.MANUAL;
                    originalState = armState;
                }

                if (((gamepad1.x != lastToggleX) && gamepad1.x) || (gamepad1.dpad_down != lastToggle_DPadDown) && gamepad1.dpad_down){
                    armState = ArmState.HIGH_LEVEL;
                }
                break;

            case HIGH_LEVEL:
                if (((gamepad1.x != lastToggleX) && gamepad1.x) || ((gamepad1.dpad_up != lastToggle_DPadUp) && gamepad1.dpad_up)){
                    armState = ArmState.HOME;
                }

                if (gamepad1.dpad_down != lastToggle_DPadDown && gamepad1.dpad_down){
                    armState = ArmState.MEDIUM_LEVEL;
                }

                if (gamepad1.right_bumper != lastToggle_RightBumper && gamepad1.right_bumper){
                    armState = ArmState.MANUAL;
                    originalState = armState;
                }

                break;
            case MEDIUM_LEVEL:
                if (gamepad1.dpad_up != lastToggle_DPadUp && gamepad1.dpad_up){
                    armState = ArmState.HIGH_LEVEL;
                }

                if (gamepad1.dpad_down != lastToggle_DPadDown && gamepad1.dpad_down){
                    armState = ArmState.LOW_LEVEL;
                }

                if (gamepad1.x != lastToggleX && gamepad1.x){
                    armState = ArmState.HOME;
                }

                if (gamepad1.right_bumper != lastToggle_RightBumper && gamepad1.right_bumper){
                    armState = ArmState.MANUAL;
                    originalState = armState;
                }

                break;
            case LOW_LEVEL:
                if (gamepad1.dpad_up != lastToggle_DPadUp && gamepad1.dpad_up){
                    armState = ArmState.MEDIUM_LEVEL;
                }

                if (gamepad1.x != lastToggleX && gamepad1.x){
                    armState = ArmState.HOME;
                }

                if (gamepad1.right_bumper != lastToggle_RightBumper && gamepad1.right_bumper){
                    armState = ArmState.MANUAL;
                    originalState = armState;
                }

                break;
            case MANUAL:
                if (gamepad1.right_bumper != lastToggle_RightBumper && gamepad1.right_bumper){
                    armState = originalState;
                }
                if (gamepad1.x != lastToggleX && gamepad1.x){
                    armState = ArmState.HOME;
                }
                break;
        }

        lastToggleX = gamepad1.x;
        lastToggle_DPadUp = gamepad1.dpad_up;
        lastToggle_DPadDown = gamepad1.dpad_down;
        lastToggle_RightBumper = gamepad1.right_bumper;
    }

    public void Telemetry(){
        telemetry.addData("ArmPos Degrees", getAngleDegrees());
        telemetry.addData("CookieMonster_Arm error", ArmPID.error);
        telemetry.addData("CookieMonster_Arm target angle", targetAngle);
        telemetry.addData("ArmState", armState);
        telemetry.addData("ArmPower", ArmPower);
    }

    public void TuningTelemetry(){
        telemetry.addData("ArmPos Degrees", getAngleDegrees());
        telemetry.addData("CookieMonster_Arm error", ArmPID.error);
        telemetry.addData("CookieMonster_Arm target angle", targetAngle);
        telemetry.addData("ArmState", armState);
        telemetry.addData("ArmPower", ArmPower);
        telemetry.addData("Anti-Gravity", antiGravity);
        telemetry.addData("Proportion", ArmPID.P);
        telemetry.addData("Integral", ArmPID.I);
        telemetry.addData("Derivative", ArmPID.D);
    }
}
