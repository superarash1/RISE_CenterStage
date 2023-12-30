package org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Control.PID_Controller;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.DoubleReverse4Bar_Old;

public class Ri3D_DR4B_Test_Control {
    public DoubleReverse4Bar_Old DR4B;

    public enum barState{
        INTAKE,
        SCORE_HIGH
    }

    boolean barToggle1 = true;
    boolean barToggle2 = false;

    boolean lastToggleX = false;

    double power = 0;

    public Telemetry telemetry;
    public Gamepad gamepad1;
    public PID_Controller BarPID;

    barState BarState;

    public Ri3D_DR4B_Test_Control(String BarMotorName, Telemetry telemetry, Gamepad gamepad1, HardwareMap hardwareMap){
        DR4B = new DoubleReverse4Bar_Old(BarMotorName, 1, hardwareMap);

        BarPID = new PID_Controller(5.25, 0.0001, 0, 0.001); //5.25, 0.0001
        BarPID.tolerance = 2;

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        BarState = barState.INTAKE;
    }

    public void DoubleReverse4Bar_Motion(){
        switch (BarState){
            case INTAKE:
                power = BarPID.PID_Power(DR4B.barMotor.getCurrPosDegrees(), 0);
                DR4B.barMotor.setPower(power);

                if ((gamepad1.x != lastToggleX) && gamepad1.x && barToggle1){
                    BarState = barState.SCORE_HIGH;

                    barToggle1 = false;
                    barToggle2 = true;
                }

                lastToggleX = gamepad1.x;

                break;

            case SCORE_HIGH:
                power = BarPID.PID_Power(DR4B.barMotor.getCurrPosDegrees(), -110);
                DR4B.barMotor.setPower(power);

                if ((gamepad1.x != lastToggleX) && gamepad1.x && barToggle2){
                    BarState = barState.INTAKE;

                    barToggle1 = true;
                    barToggle2 = false;
                }

                lastToggleX = gamepad1.x;

                break;
        }
    }

    public void Telemetry(){
        telemetry.addData("State", BarState);
        telemetry.addData("PID Power", power);
        telemetry.addData("Error", BarPID.error);
        telemetry.addData("Proportion", BarPID.P);
        telemetry.addData("Integral", BarPID.I);
        telemetry.addData("Derivative", BarPID.D);
        telemetry.addData("angle", DR4B.barMotor.getCurrPosDegrees());
    }
}
