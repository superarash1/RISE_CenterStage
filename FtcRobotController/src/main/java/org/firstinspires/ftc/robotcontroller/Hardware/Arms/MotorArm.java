package org.firstinspires.ftc.robotcontroller.Hardware.Arms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Motor;

public abstract class MotorArm {
    public Motor motors[];

    public double kGravity;
    public double antiGravity;
    public double initialAngle;
    public double gearRatio;

    public MotorArm(int motorCount, String[] name, double CPR, double gearRatio, double kGravity, double initialAngle, HardwareMap hardwareMap){

        Motor[] motors = new Motor[motorCount];

        this.motors = motors;

        for (int i = 0; i < motors.length; i++){
            motors[i] = new Motor(name[i], CPR, hardwareMap);
        }

        reset();

        for (Motor motor : motors) {
            motor.setBreakMode();
        }

        //TODO
//        leftBarMotor.setDirectionReverse();
//        rightBarMotor.setDirectionForward();

        this.gearRatio = gearRatio;
        this.kGravity = kGravity;
        this.initialAngle = initialAngle;

    }

    //GR = 0.2380952380952381
    // initialAngle = -37 degrees??? (convert to radians)

    public void setPower(double power){
        //TODO: check if gravity is exactly directly proportional with Cos(angle)
        antiGravity = kGravity*Math.cos(getAngle());

        for (Motor motor : motors) {
            motor.setPower(antiGravity + power);
        }
    }
    public double getAngle(){
        return (gearRatio*motors[0].getCurrPosRadians()) + Math.toRadians(initialAngle);
    }
    public double getAngleDegrees(){
        return (gearRatio*motors[0].getCurrPosDegrees()) + (initialAngle);
    }

    public void reset(){
        for (Motor motor : motors) {
            motor.reset();
        }
    }
}
