package org.firstinspires.ftc.team8109_Rise.Robots.Batholomew.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Control.PIDF_Controller;
import org.firstinspires.ftc.team8109_Rise.Hardware.Lifts.Slides;

public class ViperSlides extends Slides {

    Gamepad gamepad1;
    Telemetry telemetry;

    static String[] name = {"slidesLeft", "slidesRight"};

    static double pulleyRadius = 0.752;

    public double coneCount = 5;

    double slidesPower = 0;

    public enum SlidesState{
        GROUND,
        LOW_JUNCTION,
        MIDDLE_JUNCTION,
        HIGH_JUNCTION,
        CONESTACK,
        MANUAL
    }

    boolean toggle1 = true;
    boolean toggle2 = false;

    boolean toggleB1 = true;
    boolean toggleB2 = false;

    boolean triggerToggle1 = true;
    boolean triggerToggle2 = false;
    boolean toggleManual = false;
    boolean[] autonTestToggles = {true, false,  true, false,  true, false,  true, false, true, false};
    boolean[] junctionToggles = {true, false,  true, false,  true, false, true, false};

    boolean lastToggleX = false;
    boolean lastToggleB = false;
    boolean lastToggleUp = false;
    boolean lastToggleDown = false;

    boolean lastTriggerLeft = false;

    public SlidesState slidesState;

    public PIDF_Controller slidesPID;

    public double targetPos = 0;

    public ViperSlides(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super(2, name, pulleyRadius, StringingMethod.CONTINUOUS, 2, 0, hardwareMap); //0.175

        slidesPID = new PIDF_Controller(0.31, 0.03, 0, 0.01); //0.07, 0.0035, 0, 0.01

        motors[0].reset();
        motors[1].reset();

        // One of the motors needs to be reversed since the motors face opposite directions
        motors[0].setDirectionReverse();

        // Need to use the instances of gamepad1 and telemetry from the class LinearOpmode because that's what the code runs
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        // The starting state of the slides is to be in manual control
        slidesState = SlidesState.GROUND;
    }

    // Method looped to continually set power to slides based on state
    public void setSlidePower(){
        /* PID controller calculates the power needed to be set to the motors to stay at the target position  */
        slidesPower = slidesPID.PIDF_Power(getHeight(), targetPos);
        slidesPID.tolerance = 0.001;
        switch (slidesState){
            case GROUND:
                targetPos = 0;
                break;

            case HIGH_JUNCTION:
                targetPos = 18.4;
                break;

            case MIDDLE_JUNCTION:
                targetPos = 10.27;
                break;

            case LOW_JUNCTION:
                targetPos = 0.8;
                break;

            case CONESTACK:
                targetPos = (coneCount-1)*1.25;
                break;

            case MANUAL:
                if (gamepad1.dpad_up){
                    slidesPower = -0.25;
                } else if (gamepad1.dpad_down){
                    slidesPower = 0.25;
                } else {
                    slidesPower = 0;
                }

                //TODO: add manual arm control too (and maybe wrist as well)
//                if (gamepad1.dpad_right){
//                    cagePosition += 0.01;
//                } else if (gamepad1.dpad_left){
//                    cagePosition -= 0.01;
//                }
                break;
        }
        // Sets the power to the slides motors
        setPower(slidesPower);
    }

    public void toggleStates(){
        setSlidePower();
        switch (slidesState){
            case GROUND:
                if ((gamepad1.x != lastToggleX) && gamepad1.x){
                    slidesState = SlidesState.HIGH_JUNCTION;
                }

                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                    slidesState = SlidesState.LOW_JUNCTION;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b){
                    slidesState = SlidesState.MANUAL;
                }
                break;

            case LOW_JUNCTION:
                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                    slidesState = SlidesState.MIDDLE_JUNCTION;
                }

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down){
                    slidesState = SlidesState.GROUND;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b){
                    slidesState = SlidesState.MANUAL;
                }
                break;
            case MIDDLE_JUNCTION:
                if ((gamepad1.x != lastToggleX) && gamepad1.x){
                    slidesState = SlidesState.GROUND;
                }

                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                    slidesState = SlidesState.HIGH_JUNCTION;
                }

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down){
                    slidesState = SlidesState.LOW_JUNCTION;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b){
                    slidesState = SlidesState.MANUAL;
                }
                break;

                //TODO:
            case HIGH_JUNCTION:
                if ((gamepad1.x != lastToggleX) && gamepad1.x){
                    slidesState = SlidesState.GROUND;
                }

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down){
                    slidesState = SlidesState.MIDDLE_JUNCTION;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b){
                    slidesState = SlidesState.MANUAL;
                }
                break;
            case MANUAL:
                if ((gamepad1.b != lastToggleB) && gamepad1.b){
                    slidesState = SlidesState.GROUND;
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
