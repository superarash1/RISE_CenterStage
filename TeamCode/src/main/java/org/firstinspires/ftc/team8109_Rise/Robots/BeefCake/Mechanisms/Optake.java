package org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Control.PID_Controller;
import org.firstinspires.ftc.team8109_Rise.Hardware.Intakes.PassiveIntake;

public class Optake extends PassiveIntake {
    Gamepad gamepad1;
    Telemetry telemetry;
    static String[] name = {"intakeMotor"};
    public double motorPower = 0;

    public enum IntakeState {
        RUNNING,
        IDLE
    }

    IntakeState intakeState;

    PID_Controller IntakePID;

    public Optake(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super("intakeMotor", 537.7, gamepad1, telemetry, hardwareMap);

        IntakePID = new PID_Controller(0);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        intakeState = IntakeState.IDLE;
    }

    @Override
    public void toggleIntake() {
        motorPower = gamepad1.right_trigger - gamepad1.left_trigger;
        motor.setPower(motorPower);
    }

    // This function normalizes the angle so it returns a value between -180° and 180° instead of 0° to 360°.
    public double angleWrap(double degrees) {
        if (degrees > 180) {
            degrees -= 2 * 180;
        }
        if (degrees < -180) {
            degrees += 2 * 180;
        }
        // keep in mind that the result is in degrees
        return degrees;
    }
    public void Telemetry(){
        telemetry.addData("RightTrigger", gamepad1.right_trigger);
        telemetry.addData("LeftTrigger", gamepad1.left_trigger);
        telemetry.addData("motorPower", motorPower);
    }
}
