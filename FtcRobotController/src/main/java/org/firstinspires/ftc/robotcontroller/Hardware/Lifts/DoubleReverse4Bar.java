package org.firstinspires.ftc.robotcontroller.Hardware.Lifts;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Hardware.Motor;

public abstract class DoubleReverse4Bar {
    public Motor leftBarMotor;
    public Motor rightBarMotor;

    public double kGravity;
    public double antiGravity;
    public double barLength;
    public double maxExtension;
    public double initialAngle;
    public double gearRatio;

    double initialHeight;

    double DR4B_Power;

    public DoubleReverse4Bar(String leftBarMotorName, String rightBarMotorName, HardwareMap hardwareMap){
        leftBarMotor = new Motor(leftBarMotorName, 537.7, hardwareMap);
        rightBarMotor = new Motor(rightBarMotorName, 537.7, hardwareMap);

        reset();

        leftBarMotor.setBreakMode();
        rightBarMotor.setBreakMode();
    }

    public DoubleReverse4Bar(String leftBarMotorName, String rightBarMotorName, double CPR, HardwareMap hardwareMap){
        leftBarMotor = new Motor(leftBarMotorName, CPR, hardwareMap);
        rightBarMotor = new Motor(rightBarMotorName, CPR, hardwareMap);

        reset();

        leftBarMotor.setBreakMode();
        rightBarMotor.setBreakMode();
    }

    public DoubleReverse4Bar(String leftBarMotorName, String rightBarMotorName, double CPR, double kGravity, HardwareMap hardwareMap){
        leftBarMotor = new Motor(leftBarMotorName, CPR, hardwareMap);
        rightBarMotor = new Motor(rightBarMotorName, CPR, hardwareMap);

        reset();

        leftBarMotor.setBreakMode();
        rightBarMotor.setBreakMode();

        this.kGravity = kGravity;
    }

    public DoubleReverse4Bar(String leftBarMotorName, String rightBarMotorName, double CPR, double gearRatio, double kGravity, double initialHeight, HardwareMap hardwareMap){
        leftBarMotor = new Motor(leftBarMotorName, CPR, hardwareMap);
        rightBarMotor = new Motor(rightBarMotorName, CPR, hardwareMap);

        reset();

        leftBarMotor.setBreakMode();
        rightBarMotor.setBreakMode();

        this.gearRatio = gearRatio;
        this.kGravity = kGravity;
        this.initialHeight = initialHeight;
    }

    public DoubleReverse4Bar(String leftBarMotorName, String rightBarMotorName, double CPR, double gearRatio, double kGravity, double initialHeight, double maxExtension, double barLength, double initialAngle, HardwareMap hardwareMap){
        leftBarMotor = new Motor(leftBarMotorName, CPR, hardwareMap);
        rightBarMotor = new Motor(rightBarMotorName, CPR, hardwareMap);

        reset();

        leftBarMotor.setBreakMode();
        rightBarMotor.setBreakMode();

        leftBarMotor.setDirectionReverse();
        rightBarMotor.setDirectionForward();

        this.gearRatio = gearRatio;
        this.kGravity = kGravity;
        this.initialHeight = initialHeight;
        this.maxExtension = maxExtension;
        this.barLength = barLength;
        this.initialAngle = initialAngle;
    }

    public void setPower(double power){
        //TODO: check if gravity is exactly directly proportional with Cos(angle)
        antiGravity = kGravity*Math.cos(getAngle());
        leftBarMotor.setPower(antiGravity + power);
        rightBarMotor.setPower(antiGravity + power);
    }

    public double getAngle(){
        return (gearRatio*leftBarMotor.getCurrPosRadians()) + Math.toRadians(initialAngle);
    }

    public double getHeight(){
        return 2*barLength*Math.sin(getAngle()) + initialHeight;
    }

    public void reset(){
        leftBarMotor.reset();
        rightBarMotor.reset();
    }
}
