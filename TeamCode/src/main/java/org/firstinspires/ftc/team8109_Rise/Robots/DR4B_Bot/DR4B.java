package org.firstinspires.ftc.team8109_Rise.Robots.DR4B_Bot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Control.PIDF_Controller;
import org.firstinspires.ftc.team8109_Rise.Hardware.Lifts.DoubleReverse4Bar;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.Claw;

public class DR4B extends DoubleReverse4Bar {
    static double GearRatio = 0.2;
    static double CPR = 537.7;

    public Gamepad gamepad1;
    public Telemetry telemetry;

    public Claw claw;

    double clawPosition;

    boolean gateToggle1 = true;
    boolean gateToggle2 = false;

    boolean lastToggleY = true;

    static double kGravity = 0.225;
    static double initialHeight = 12.3;
    static double maxExtension = 39;
    static double barLength = 12.5;
    static double initialAngle = -30.1;

    public double DR4B_Power;
    public double targetHeight;

    boolean barToggle1 = true;
    boolean barToggle2 = false;

    boolean lastToggleX = false;

    public enum liftState{
        HOME,
        LOW_JUNCTION,
        MIDDLE_JUNCTION,
        HIGH_JUNCTION,
        MANUAL
    }

    liftState LiftState;

    public PIDF_Controller DR4B_PID;

    public DR4B(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super("dr4bLeft", "dr4bRight",
                CPR, GearRatio, kGravity, initialHeight, maxExtension, barLength, initialAngle,
                hardwareMap);

        claw = new Claw("claw", hardwareMap);
        DR4B_PID = new PIDF_Controller(0.025, 0.007, 0, 0.008); //0.015
        LiftState = liftState.MANUAL;

        DR4B_PID.tolerance = 0.5;

        clawPosition = 0.55;

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void Set_DR4B_Power(){
        DR4B_Power = DR4B_PID.PIDF_Power(getHeight(), targetHeight);
        switch (LiftState){
            case HOME:
                targetHeight = 4.5;
                break;

            case HIGH_JUNCTION:
                targetHeight = 36.5;
                break;

                // 1.5"
            // 36.75
                //TODO: finish other drv4b levels
            case MANUAL:
                if (gamepad1.dpad_up && (getHeight() < 36.5)){
                    DR4B_Power = 0.25;
                } else if (gamepad1.dpad_down){
                    DR4B_Power = -0.2;
                  } else {
                    DR4B_Power = 0;
                    }

                break;
        }
        setPower(DR4B_Power);
    }

    public void DriverControl(){
        switch (LiftState){
            case HOME:
                if ((gamepad1.x != lastToggleX) && gamepad1.x && barToggle1){
                    barToggle1 = false;
                    barToggle2 = true;

                    LiftState = liftState.HIGH_JUNCTION;
                }
                break;

            case HIGH_JUNCTION:
                if ((gamepad1.x != lastToggleX) && gamepad1.x && barToggle2){
                    barToggle2 = false;
                    barToggle1 = true;

                    LiftState = liftState.HOME;
                }
                break;

            //TODO: finish other drv4b levels
            case MANUAL:

                break;
        }
        lastToggleX = gamepad1.x;
    }

    public void Claw(){
        claw.clawServo.setPosition(clawPosition);

        if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle1){
            gateToggle1 = false;
            gateToggle2 = true;

            clawPosition = 0.55;
        }else if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle2){
            gateToggle2 = false;
            gateToggle1 = true;

            clawPosition = 0.375;
        }

        lastToggleY = gamepad1.y;

    }

    public void Telemetry(){
        telemetry.addData("claw position", claw.clawServo.getPosition());
        telemetry.addData("drv4b angle", getAngle());
        telemetry.addData("drv4b Target Height", targetHeight);
        telemetry.addData("drv4b Height", getHeight());
        telemetry.addData("antiGravity", antiGravity);
        telemetry.addData("drv4b power", DR4B_Power);
        telemetry.update();
    }
}
