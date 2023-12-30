package org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Control.PID_Controller;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.MecanumDriveTrain_Old;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.Slides;

public class SlidesTestControl {
    Slides drawerSlides;
    MecanumDriveTrain_Old driveTrain;

    Gamepad gamepad1;
    Telemetry telemetry;

    double drive;
    double turn;
    double strafe;
    double fLeft;
    double fRight;
    double bLeft;
    double bRight;
    double max;


    boolean slidesToggle1 = true;
    boolean slidesToggle2 = false;
    boolean slidesToggle3 = true;
    boolean slidesToggle4 = false;

    boolean lastToggleDown = true;
    boolean lastToggleUp = true;

    public enum slideState{
        IDLE,
        HOME,
        FIVE_INCHES,
        TEN_INCHES,
        TOP_LEVEL
    }

    double height = 10;

    PID_Controller slidesPID;
    slideState SlideState;

    public SlidesTestControl(String flName, String frName, String brName, String blName, Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        drawerSlides = new Slides("slides", 1, 0.44690708020204210283902560755002, height, 0,  hardwareMap);

        driveTrain = new MecanumDriveTrain_Old(flName, frName, brName, blName, hardwareMap);

        driveTrain.setBreakMode();
        driveTrain.reset();


        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        slidesPID = new PID_Controller(0.5);
        SlideState = slideState.IDLE;
    }

    public void SlidesControl(){
        switch (SlideState){
            case IDLE:
                drawerSlides.slidesMotor.setPower(0);

                if (gamepad1.x){
                    SlideState = slideState.HOME;
                }

                break;
            case HOME:
                drawerSlides.slidesMotor.setPower(slidesPID.PID_Power(drawerSlides.getHeight(), 0));

                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up && slidesToggle1){
                    slidesToggle1 = false;
                    slidesToggle2 = true;

                    SlideState = slideState.FIVE_INCHES;
                }

                lastToggleUp = gamepad1.dpad_up;

                break;
            case FIVE_INCHES:
                drawerSlides.slidesMotor.setPower(slidesPID.PID_Power(drawerSlides.getHeight(), 5));

                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up && slidesToggle3){
                    slidesToggle3 = false;
                    slidesToggle4 = true;

                    SlideState = slideState.FIVE_INCHES;
                }

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down && slidesToggle2){
                    slidesToggle2 = false;
                    slidesToggle1 = true;

                    SlideState = slideState.HOME;
                }

                lastToggleUp = gamepad1.dpad_up;
                lastToggleDown = gamepad1.dpad_down;

                break;

            case TEN_INCHES:
                drawerSlides.slidesMotor.setPower(slidesPID.PID_Power(drawerSlides.getHeight(), 10));

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down && slidesToggle4){
                    slidesToggle4 = false;
                    slidesToggle3 = true;

                    SlideState = slideState.FIVE_INCHES;
                }

                lastToggleDown = gamepad1.dpad_down;

                break;
        }
    }

    public void Telemetry(){
        telemetry.addData("Slides Height", drawerSlides.getHeight());
    }
}
