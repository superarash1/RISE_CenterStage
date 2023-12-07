package org.firstinspires.ftc.team8109_Rise.OldCode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team8109_Rise.Hardware.Motor;

public class Slides {
    public Motor slidesMotor;
    public double pulleyDiameter;
    public double maxHeight;
    public double kGravity = 0;

    public Slides(String Slides, double gearRatio, double pulleyDiameter, double maxHeight, HardwareMap hardwareMap){
        slidesMotor = new Motor(Slides, 537.7, hardwareMap); //0.44690708020204210283902560755002
        this.pulleyDiameter = pulleyDiameter;
        this.maxHeight = maxHeight;
    }

    public Slides(String Slides, double gearRatio, double pulleyDiameter, double maxHeight, double kGravity, HardwareMap hardwareMap){
        slidesMotor = new Motor(Slides, 537.7, hardwareMap); //0.44690708020204210283902560755002
        this.pulleyDiameter = pulleyDiameter;
        this.maxHeight = maxHeight;
        this.kGravity = kGravity;
    }

    public void setPower(double power){
        slidesMotor.setPower(kGravity + power);
    }

    public double getHeight(){
        return Math.toRadians(slidesMotor.getCurrPosDegrees()) * pulleyDiameter;
    }
}
