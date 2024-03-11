package org.firstinspires.ftc.robotcontroller.Hardware.Intakes;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcontroller.Hardware.Motor;

public abstract class PassiveIntake {
    Gamepad gamepad1;
    Telemetry telemetry;
    public Motor motors[];
    public Motor motor;
    //537.7


    public PassiveIntake(int motorCount, String[] name, double cpr, Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        Motor motors[] = new Motor[motorCount];
        for (int i = 0; i <motors.length; i++){
            motors[i] = new Motor(name[i], cpr, hardwareMap);
        }

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        this.motors = motors;
    }

    public PassiveIntake(String name, double cpr, Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        motor = new Motor(name, cpr, hardwareMap);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void toggleIntake(){
        for (Motor motor : motors) {
            motor.setPower(Math.pow(3, gamepad1.right_trigger - gamepad1.left_trigger));
            motor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }
    }

    //TODO: make intake position method using encoders
}
