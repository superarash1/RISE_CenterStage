package org.firstinspires.ftc.robotcontroller.Hardware.Arms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class ServoArm {
    public Servo armServo1;
    public Servo armServo2;

    public enum ServoArmType{
        SINGLE_SERVO,
        DOUBLE_SERVO
    }

    public enum AngleUnit {
        DEGREES,
        RADIANS
    }

    ServoArmType servoArmType;

    public ServoArm(ServoArmType servoArmType, String[] name, HardwareMap hardwareMap){
        armServo1 = hardwareMap.get(Servo.class, name[0]);
        armServo2 = hardwareMap.get(Servo.class, name[1]);

        armServo2.setDirection(Servo.Direction.REVERSE);

        this.servoArmType = servoArmType;
    }

    public double getPositionDegrees(){
        return armServo1.getPosition()*300;
    }

    public double getPositionRadians(){
        return armServo1.getPosition()*((3*Math.PI)/2);
    }

    public void setAngle(double angle){
        armServo1.setPosition(angle/300);
        armServo2.setPosition(angle/300);
    }

    public void setAngle(double angle, AngleUnit unit){
        if (unit == AngleUnit.DEGREES) {
            armServo1.setPosition(angle/300);
            armServo2.setPosition(angle/300);
        }

        if (unit == AngleUnit.RADIANS) {
            armServo1.setPosition(angle/((5*Math.PI)/3));
            armServo2.setPosition(angle/((5*Math.PI)/3));
        }
    }


}
