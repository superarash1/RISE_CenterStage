package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Control.PID_Controller;
import org.firstinspires.ftc.robotcontroller.Hardware.Arms.MotorArm;
import org.firstinspires.ftc.robotcontroller.Hardware.Lifts.Slides;
import org.firstinspires.ftc.robotcontroller.Hardware.Motor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class TenacityArm extends MotorArm {
    Gamepad gamepad1;
    Telemetry telemetry;
    static String[] names = {"leftArm", "rightArm"};

    double ArmPower = 0;
    double targetAngle;


    public enum ArmState {
        INIT,
        CLOSE_INTAKE,
        FAR_INTAKE,
        FIRST_LINE,
        SECOND_LINE,
        THIRD_LINE,
        HANG,
        MANUAL
    }

    public PID_Controller ArmPID;

    public ArmState armState;

    public static double arm_kp = 0.045; //0.01
    public static double arm_kd = 0;
    public static double arm_a = 0;
    public static double arm_ki = 0;

    public static double angle = -15;

    Slides slides;
    public TenacityArm(Slides slides, Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super(2, names, 28, 0.0091, 0.1, 45, hardwareMap);

//        motors[0].setDirectionReverse();
//        motors[1].setDirectionReverse();

        //0.0001
        ArmPID = new PID_Controller(arm_kp, arm_kd, arm_a, arm_ki);

        targetAngle = 45;

        armState = ArmState.INIT;

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        this.slides = slides;
    }

    public void setArmPower(){
        ArmPower = ArmPID.PID_Power(getAngleDegrees(), targetAngle);
        switch (armState){
            case INIT:
                targetAngle = 34;
                break;
            case CLOSE_INTAKE:
                targetAngle = angle;
                break;
            case FAR_INTAKE:
                targetAngle = -12;
                break;
            case FIRST_LINE:
                targetAngle = 142;
                break;

            case SECOND_LINE:
                targetAngle = 123;
                break;

            case THIRD_LINE:
                targetAngle = 110;
                break;

            case MANUAL:
                if (gamepad1.dpad_up){
                    ArmPower = -0.25;
                } else if (gamepad1.dpad_down){
                    ArmPower = 0.25;
                } else {
                    ArmPower = 0;
                }
                break;
        }
        setPower(ArmPower);
    }

    @Override
    public double getAngleDegrees() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.thirdAngle;
    }

    public void setPower(double power){
        //TODO: check if gravity is exactly directly proportional with Cos(angle)
        antiGravity = kGravity*Math.cos(Math.toRadians(getAngleDegrees()))*(slides.getExtension()+8);

        double totalPower = antiGravity + power;
        for (Motor motor : motors){
            motor.setPower(-totalPower);
        }
    }

    public void Telemetry(){
        telemetry.addData("ArmPos Degrees", getAngleDegrees());
        telemetry.addData("CookieMonster_Arm error", ArmPID.error);
        telemetry.addData("CookieMonster_Arm target angle", targetAngle);
        telemetry.addData("ArmState", armState);
        telemetry.addData("ArmPower", ArmPower);
    }

    public void TuningTelemetry(){
        telemetry.addData("Real Arm Power", antiGravity+ArmPower);
        telemetry.addData("ArmPos Degrees", getAngleDegrees());
        telemetry.addData("Arm error", ArmPID.error);
        telemetry.addData("target angle", targetAngle);
        telemetry.addData("ArmState", armState);
        telemetry.addData("ArmPower", ArmPower);
        telemetry.addData("Anti-Gravity", antiGravity);
        telemetry.addData("Proportion", ArmPID.P);
        telemetry.addData("Integral", ArmPID.I);
        telemetry.addData("Derivative", ArmPID.D);
    }
}
