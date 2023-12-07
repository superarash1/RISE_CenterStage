package org.firstinspires.ftc.team8109_Rise.OldCode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team8109_Rise.Hardware.Motor;

public class DoubleReverse4Bar_Old {
    public Motor barMotor;
    public double kGravity;
    public double antiGravity;
    public double barLength;
    public double maxExtension;
    public double initialAngle;

    double initialHeight;

    public double motorPower = 0;

    public DoubleReverse4Bar_Old(String barMotorName, double GearRatio, HardwareMap hardwareMap){
        // TODO: move cpr to constructor
        barMotor = new Motor(barMotorName, 537.7, hardwareMap);
        barMotor.reset();
        barMotor.setBreakMode();
    }

    public DoubleReverse4Bar_Old(String barMotorName, double GearRatio, double kGravity, HardwareMap hardwareMap){
        barMotor = new Motor(barMotorName, 537.7, hardwareMap);
        barMotor.reset();
        barMotor.setBreakMode();

        this.kGravity = kGravity;
    }

    public DoubleReverse4Bar_Old(String barMotorName, double GearRatio, double kGravity, double initialHeight, HardwareMap hardwareMap){
        barMotor = new Motor(barMotorName, hardwareMap);
        barMotor.reset();
        barMotor.setBreakMode();

        this.kGravity = kGravity;
        this.initialHeight = initialHeight;
    }

    public DoubleReverse4Bar_Old(String barMotorName, double GearRatio, double kGravity, double initialHeight, double maxExtension, double barLength, double initialAngle, HardwareMap hardwareMap){
        barMotor = new Motor(barMotorName, hardwareMap);
        barMotor.reset();
        barMotor.setBreakMode();

        this.kGravity = kGravity;
        this.initialHeight = initialHeight;
        this.maxExtension = maxExtension;
        this.barLength = barLength;
        this.initialAngle = initialAngle;
    }

    public void setPower(double power){
        //TODO: actually do the math
//        antiGravity = kGravity * Math.cos((1/(maxExtension*Math.PI))*getHeight());
//        antiGravity = kGravity*Math.sqrt((barLength*barLength) - ((getHeight()/2)*(getHeight()/2)));
//        antiGravity = (kGravity*barLength)/(2*Math.sin(getAngle()));
        antiGravity = kGravity*Math.cos(getAngle());
        barMotor.setPower(antiGravity + power);
    }

    public double getAngle(){
        return barMotor.getCurrPosRadians() + Math.toRadians(initialAngle);
    }

    public double getHeight(){
        // 4 + (barMotor.getCurrentPosition()*kPos)
        return 2*barLength*Math.sin(getAngle()) + initialHeight; // convert to variable
    }
}
