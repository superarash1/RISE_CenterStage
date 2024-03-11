package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Arms.ServoArm;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.ViperSlides;

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
            case DUNK_POSITION:
                setAngle(264);
                break;
            case AUTO_START:
                setAngle(252);
                break;
            case AUTO_NEW:
                setAngle(90);
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
    public void togglePosition(){
        setArmPosition();
        switch (servoPosition){
            case INTAKE_POSITION:
                if ((gamepad1.x != lastToggleX) && gamepad1.x && toggle1){
                    toggle1 = false;
                    toggle2 = true;

                    servoPosition = ServoPosition.OUTTAKE_POSITION;
                }

                break;

            case OUTTAKE_POSITION:
                if ((gamepad1.x != lastToggleX) && gamepad1.x && toggle2){
                    toggle2 = false;
                    toggle1 = true;

                    servoPosition = ServoPosition.INTAKE_POSITION;
                }

                if ((gamepad1.left_bumper != lastTriggerLeft) && gamepad1.left_bumper && triggerToggle1){
                    triggerToggle1 = false;
                    triggerToggle2 = true;

                    servoPosition = ServoPosition.DUNK_POSITION;
                }
                break;
            case DUNK_POSITION:
                if ((gamepad1.x != lastToggleX) && gamepad1.x && toggle2){
                    toggle2 = false;
                    toggle1 = true;

                    triggerToggle1 = false;
                    triggerToggle2 = true;
                    servoPosition = ServoPosition.INTAKE_POSITION;
                }

                if ((gamepad1.left_bumper != lastTriggerLeft) && gamepad1.left_bumper && triggerToggle2){
                    triggerToggle2 = false;
                    triggerToggle1 = true;
                    servoPosition = ServoPosition.OUTTAKE_POSITION;
                }
                break;
        }
        lastToggleX = gamepad1.x;
        lastTriggerLeft = gamepad1.left_bumper;
    }

    public void slidesToggle(ViperSlides.SlidesState slidesState){
        setArmPosition();
        switch (slidesState){
            case GROUND:
                servoPosition = ServoPosition.INTAKE_POSITION;
                break;

            case LOW_JUNCTION:
                servoPosition = ServoPosition.OUTTAKE_POSITION;
                break;
            case MIDDLE_JUNCTION:
                servoPosition = ServoPosition.OUTTAKE_POSITION;
                break;

            case HIGH_JUNCTION:
                servoPosition = ServoPosition.OUTTAKE_POSITION;
                break;

            case LOW_DUNK:
                servoPosition = ServoPosition.DUNK_POSITION;
                break;
            case MIDDLE_DUNK:
                servoPosition = ServoPosition.DUNK_POSITION;
                break;
            case HIGH_DUNK:
                servoPosition = ServoPosition.DUNK_POSITION;
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
