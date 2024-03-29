package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Control.PID_Controller;
import org.firstinspires.ftc.robotcontroller.Hardware.Lifts.Slides;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TenacitySlides extends Slides {

    Gamepad gamepad1;
    Telemetry telemetry;

    static String[] name = {"slidesLeft", "slidesRight"};
    static String[] armName = {"armLeft", "armRight"};

    static double pulleyRadius = 1.5*((double) 16 /46); //16/46

    double slidesPower = 0;

    public enum SlidesState{
        HOME,
        FIRST_LINE,
        SECOND_LINE,
        THIRD_LINE,
        HANG,
        MANUAL
    }

    public SlidesState slidesState;
    public PID_Controller slidesPID;

    public double targetPos = 0;

    public TenacitySlides(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super(2, name, pulleyRadius, StringingMethod.CASCADE, 2, 0, hardwareMap); //0.175

        slidesPID = new PID_Controller(0, 0, 0, 0); //0.07, 0.0035, 0, 0.01

        motors[0].reset();
        motors[1].reset();

        // One of the motors needs to be reversed since the motors face opposite directions
//        motors[0].setDirectionReverse();

        // Need to use the instances of gamepad1 and telemetry from the class LinearOpmode because that's what the code runs
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        // The starting state of the slides is to be in manual control
        slidesState = SlidesState.HOME;
    }

    // Method looped to continually set power to slides based on state
    public void setSlidePower(){
        /* PID controller calculates the power needed to be set to the motors to stay at the target position  */
        slidesPID.tolerance = 0.001;

        switch (slidesState){
            case HOME:
                targetPos = 0;
                slidesPower = slidesPID.PID_Power(getExtension(), targetPos);
                break;

            case FIRST_LINE:
                targetPos = 6;
                slidesPower = slidesPID.PID_Power(getExtension(), targetPos);
                break;

            case THIRD_LINE:
                targetPos = 13;
                slidesPower = slidesPID.PID_Power(getExtension(), targetPos);
                break;

            case MANUAL:
                slidesPower = gamepad1.right_trigger - gamepad1.left_trigger;
                break;
        }
        // Sets the power to the slides motors
        setPower(slidesPower);
    }

    // Sends telemetry data for slides to a queue to be shown on driver station after telemetry.update() is called
    public void slidesTelemetry(){
        telemetry.addData("Slides Height", getExtension());
        telemetry.addData("Slide State", slidesState);
        telemetry.addData("Error", slidesPID.error);
        telemetry.addData("PID Power", slidesPower);
    }

    public void tuningTelemetry(){
        telemetry.addData("slides height", getExtension());
        telemetry.addData("Slide State", slidesState);
        telemetry.addData("Error", slidesPID.error);
        telemetry.addData("Anti-Gravity", kGravity);
        telemetry.addData("PID Power", slidesPower);
        telemetry.addData("Proportion", slidesPID.P);
        telemetry.addData("Integral", slidesPID.I);
        telemetry.addData("Derivative", slidesPID.D);
    }
}
