package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Control.PID_Controller;
import org.firstinspires.ftc.robotcontroller.Hardware.Lifts.Slides;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bartholomew_Slides extends Slides {

    Gamepad gamepad1;
    Telemetry telemetry;

    public Bartholomew_Arm arm;
    public Bartholomew_Wrist wrist;
    static String[] name = {"slidesLeft", "slidesRight"};

    static double pulleyRadius = 0.752;

    double slidesPower = 0;

    public enum SlidesState{
        GROUND,
        LOW_JUNCTION,
        MIDDLE_JUNCTION,
        HIGH_JUNCTION,
        CONESTACK_FIVE,
        CONESTACK_FOUR,
        CONESTACK_THREE,
        CONESTACK_TWO,
        CONESTACK_ONE,
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

    public Bartholomew_Slides(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap) {
        super(2, name, pulleyRadius, StringingMethod.CONTINUOUS, 2, 0, hardwareMap); //0.175

        arm = new Bartholomew_Arm(gamepad1, telemetry, hardwareMap);
        wrist = new Bartholomew_Wrist(gamepad1, telemetry, hardwareMap);

        slidesPID = new PID_Controller(0.31, 0.03, 0, 0.01); //0.07, 0.0035, 0, 0.01

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
        slidesPower = slidesPID.PID_Power(getHeight(), targetPos);
        slidesPID.tolerance = 0.001;
        switch (slidesState){
            case GROUND:
                targetPos = 0;
                arm.servoPosition = Bartholomew_Arm.ServoPosition.INTAKE_POSITION;
                wrist.wristPosition = Bartholomew_Wrist.WristPosition.INTAKE_POSITION;
                break;

            case HIGH_JUNCTION:
                targetPos = 18.4;
                arm.servoPosition = Bartholomew_Arm.ServoPosition.OUTTAKE_POSITION;
                wrist.wristPosition = Bartholomew_Wrist.WristPosition.OUTTAKE_POSITION;
                break;

            case MIDDLE_JUNCTION:
                targetPos = 10.27;
                arm.servoPosition = Bartholomew_Arm.ServoPosition.OUTTAKE_POSITION;
                wrist.wristPosition = Bartholomew_Wrist.WristPosition.OUTTAKE_POSITION;
                break;

            case LOW_JUNCTION:
                targetPos = 0.8;
                arm.servoPosition = Bartholomew_Arm.ServoPosition.OUTTAKE_POSITION;
                wrist.wristPosition = Bartholomew_Wrist.WristPosition.OUTTAKE_POSITION;
                break;

            case CONESTACK_FIVE:
                targetPos = 5;
                arm.servoPosition = Bartholomew_Arm.ServoPosition.INTAKE_POSITION;
                wrist.wristPosition = Bartholomew_Wrist.WristPosition.OUTTAKE_POSITION;
                break;
            case CONESTACK_FOUR:
                targetPos = 3.75;
                arm.servoPosition = Bartholomew_Arm.ServoPosition.INTAKE_POSITION;
                wrist.wristPosition = Bartholomew_Wrist.WristPosition.OUTTAKE_POSITION;
                break;
            case CONESTACK_THREE:
                targetPos = 2.5;
                arm.servoPosition = Bartholomew_Arm.ServoPosition.INTAKE_POSITION;
                wrist.wristPosition = Bartholomew_Wrist.WristPosition.OUTTAKE_POSITION;
                break;
            case CONESTACK_TWO:
                targetPos = 1.25;
                arm.servoPosition = Bartholomew_Arm.ServoPosition.INTAKE_POSITION;
                wrist.wristPosition = Bartholomew_Wrist.WristPosition.OUTTAKE_POSITION;
                break;
            case MANUAL:
                if (gamepad1.dpad_up){
                    slidesPower = -0.25;
                } else if (gamepad1.dpad_down){
                    slidesPower = 0.25;
                } else {
                    slidesPower = 0;
                }
                arm.servoPosition = Bartholomew_Arm.ServoPosition.MANUAL;
                wrist.wristPosition = Bartholomew_Wrist.WristPosition.MANUAL;
                break;
        }
        // Sets the power to the slides motors
        setPower(slidesPower);
        arm.setArmPosition();
        wrist.setPosition();
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
