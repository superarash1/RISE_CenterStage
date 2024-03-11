package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcontroller.Control.PID_Controller;
import org.firstinspires.ftc.robotcontroller.Hardware.Lifts.Slides;

public class BeefySlides extends Slides {

    Gamepad gamepad1;
    Telemetry telemetry;

    static String[] name = {"slidesLeft", "slidesRight"};
    static String[] armName = {"armLeft", "armRight"};

    static double pulleyRadius = 0.752;

    double slidesPower = 0;

    Bicep bicep;

    public enum SlidesState{
        HOME,
        FIRST_LINE,
        THIRD_LINE,
        MANUAL
    }

    boolean lastToggleX = false;
    boolean lastToggleB = false;
    boolean lastToggleUp = false;
    boolean lastToggleDown = false;

    boolean lastTriggerLeft = false;

    public SlidesState slidesState;

    public PID_Controller slidesPID;

    public double targetPos = 0;

    public BeefySlides(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super(2, name, pulleyRadius, StringingMethod.CONTINUOUS, 2, 0, hardwareMap); //0.175

        slidesPID = new PID_Controller(0, 0, 0, 0); //0.07, 0.0035, 0, 0.01

        motors[0].reset();
        motors[1].reset();

        // One of the motors needs to be reversed since the motors face opposite directions
        motors[0].setDirectionReverse();

        bicep = new Bicep(armName, hardwareMap);

        // Need to use the instances of gamepad1 and telemetry from the class LinearOpmode because that's what the code runs
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        // The starting state of the slides is to be in manual control
        slidesState = SlidesState.HOME;
    }

    // Method looped to continually set power to slides based on state
    public void setSlidePower(){
        /* PID controller calculates the power needed to be set to the motors to stay at the target position  */
        slidesPower = slidesPID.PID_Power(getHeight(), targetPos);
        slidesPID.tolerance = 0.001;

        bicep.bicepStates();
        switch (slidesState){
            case HOME:
                targetPos = 0;
                bicep.bicepState = Bicep.BicepStates.HOME;
                break;

            case FIRST_LINE:
                targetPos = 6;
                bicep.bicepState = Bicep.BicepStates.FIRST_LINE;
                break;

            case THIRD_LINE:
                targetPos = 13;
                bicep.bicepState = Bicep.BicepStates.THIRD_LINE;
                break;

            case MANUAL:
                if (gamepad1.dpad_up){
                    slidesPower = -0.25;
                } else if (gamepad1.dpad_down){
                    slidesPower = 0.25;
                } else {
                    slidesPower = 0;
                }

                bicep.bicepState = Bicep.BicepStates.MANUAL;
                break;
        }
        // Sets the power to the slides motors
        setPower(slidesPower);
    }


    public void toggleStates(){
        setSlidePower();
        switch (slidesState){
            case HOME:
                if ((gamepad1.x != lastToggleX) && gamepad1.x){
                    slidesState = SlidesState.FIRST_LINE;
                }

                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                    slidesState = SlidesState.FIRST_LINE;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b){
                    slidesState = SlidesState.MANUAL;
                }
                break;

            case FIRST_LINE:
                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                    slidesState = SlidesState.THIRD_LINE;
                }

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down){
                    slidesState = SlidesState.HOME;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b){
                    slidesState = SlidesState.MANUAL;
                }
                break;
            case THIRD_LINE:
                if ((gamepad1.x != lastToggleX) && gamepad1.x){
                    slidesState = SlidesState.HOME;
                }

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down){
                    slidesState = SlidesState.FIRST_LINE;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b){
                    slidesState = SlidesState.MANUAL;
                }
                break;
            case MANUAL:
                if ((gamepad1.b != lastToggleB) && gamepad1.b){
                    slidesState = SlidesState.HOME;
                }
                break;
        }

        lastToggleX = gamepad1.x;
        lastToggleB = gamepad1.b;
        lastToggleUp = gamepad1.dpad_up;
        lastToggleDown = gamepad1.dpad_down;
        lastTriggerLeft = gamepad1.left_bumper;
    }

    // Sends telemetry data for slides to a queue to be shown on driver station after telemetry.update() is called
    public void slidesTelemetry(){
        telemetry.addData("Slides Height", getHeight());
        telemetry.addData("Slide State", slidesState);
        telemetry.addData("Error", slidesPID.error);
        telemetry.addData("PID Power", slidesPower);
    }

    public void tuningTelemetry(){
        telemetry.addData("slides height", getHeight());
        telemetry.addData("Slide State", slidesState);
        telemetry.addData("Error", slidesPID.error);
        telemetry.addData("Anti-Gravity", kGravity);
        telemetry.addData("PID Power", slidesPower);
        telemetry.addData("Proportion", slidesPID.P);
        telemetry.addData("Integral", slidesPID.I);
        telemetry.addData("Derivative", slidesPID.D);
    }
}
