package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Arms.ServoArm;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bartholomew_Arm extends ServoArm {
    static String[] name = {"armLeft", "armRight"};

    Gamepad gamepad1;
    Telemetry telemetry;

    boolean toggle1 = true;
    boolean toggle2 = false;

    boolean triggerToggle1 = true;
    boolean triggerToggle2 = false;

    boolean lastToggleX = false;
    boolean lastTriggerLeft = false;
    public enum ServoPosition {
        INTAKE_POSITION,
        OUTTAKE_POSITION,
        DUNK_POSITION,
        AUTO_START,
        AUTO_NEW,
        MANUAL
    }

    public ServoPosition servoPosition;

    public Bartholomew_Arm(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super(ServoArmType.DOUBLE_SERVO, name, hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        servoPosition = ServoPosition.INTAKE_POSITION;
    }
    public void setArmPosition(){
        switch (servoPosition){
            case INTAKE_POSITION:
                setAngle(90);
                break;
            case OUTTAKE_POSITION:
                setAngle(257);
                break;
            case MANUAL:
                if (gamepad1.right_bumper){
                    setAngle(getPositionDegrees()+1);
                } else if (gamepad1.left_bumper){
                    setAngle(getPositionDegrees()-1);
                }
                break;
        }
    }

    public void setTelemetry(){
        telemetry.addData("Servo Angle", getPositionDegrees());
        telemetry.addData("Arm State", servoPosition);
        telemetry.addData("Raw Position", armServo1.getPosition());
        telemetry.addData("Raw Position", armServo2.getPosition());
    }
}
