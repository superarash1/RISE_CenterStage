package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.TenacityArm;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.TenacitySlides;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.TenacityWrist;


@TeleOp
public class ArmTesting extends LinearOpMode {
    TenacityArm arm;
    TenacityWrist wrist;
    TenacitySlides slides;

    public enum TeleOpState{
        HOME,
        FIRST_LINE,
        SECOND_LINE,
        THIRD_LINE,
        HANG,
        MANUAL
    }

    TeleOpState teleOpState;
    TeleOpState scoringState;

    boolean lastToggleX = false;
    boolean lastToggleUp = false;
    boolean lastToggleDown = false;
    boolean lastToggleB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        slides = new TenacitySlides(gamepad1, telemetry, hardwareMap);
        arm = new TenacityArm(slides, gamepad1, telemetry, hardwareMap);
        wrist = new TenacityWrist(gamepad1, telemetry, hardwareMap);

        teleOpState = TeleOpState.HOME;
        scoringState = TeleOpState.FIRST_LINE;

        while (opModeInInit()){
//            arm.setArmPower();

            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()){
            arm.setArmPower();
            wrist.setWristPosition();

            switch (teleOpState){
                case HOME:
                    arm.armState = TenacityArm.ArmState.CLOSE_INTAKE;

                    if ((gamepad1.x != lastToggleX) && gamepad1.x){
                        teleOpState = scoringState;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b){
                        teleOpState = TeleOpState.MANUAL;
                    }
                    break;
                case FIRST_LINE:
                    arm.armState = TenacityArm.ArmState.FIRST_LINE;

                    if ((gamepad1.x != lastToggleX) && gamepad1.x){
                        scoringState = teleOpState;
                        teleOpState = TeleOpState.HOME;
                    }

                    if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                        teleOpState = TeleOpState.SECOND_LINE;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b){
                        teleOpState = TeleOpState.MANUAL;
                    }
                    break;
                case SECOND_LINE:
                    arm.armState = TenacityArm.ArmState.SECOND_LINE;

                    if ((gamepad1.x != lastToggleX) && gamepad1.x){
                        scoringState = teleOpState;
                        teleOpState = TeleOpState.HOME;
                    }

                    if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up){
                        teleOpState = TeleOpState.THIRD_LINE;
                    }

                    if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down){
                        teleOpState = TeleOpState.FIRST_LINE;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b){
                        teleOpState = TeleOpState.MANUAL;
                    }
                    break;
                case THIRD_LINE:
                    arm.armState = TenacityArm.ArmState.THIRD_LINE;

                    if ((gamepad1.x != lastToggleX) && gamepad1.x){
                        scoringState = teleOpState;
                        teleOpState = TeleOpState.HOME;
                    }

                    if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down){
                        teleOpState = TeleOpState.SECOND_LINE;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b){
                        teleOpState = TeleOpState.MANUAL;
                    }
                    break;
                case HANG:
                    // TODO: Figure out how the hang is gonna work
                    break;
                case MANUAL:

                    if ((gamepad1.x != lastToggleX) && gamepad1.x){
                        scoringState = teleOpState;
                        teleOpState = TeleOpState.HOME;
                    }
                    break;
            }
            // TODO: Plane launcher

            lastToggleX = gamepad1.x;
            lastToggleUp = gamepad1.dpad_up;
            lastToggleDown = gamepad1.dpad_down;
            lastToggleB = gamepad1.b;

            arm.TuningTelemetry();

            slides.getExtension();

            telemetry.update();
        }
    }
}
