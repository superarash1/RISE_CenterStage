package org.firstinspires.ftc.team8109_Rise.OpModes.TeleOp_Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.TenacityArm;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.TenacitySlides;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.TenacityChassis;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.TenacityClaw;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.TenacityWrist;

@TeleOp
public class Tenacity_TeleOp extends LinearOpMode {
    TenacityChassis chassis;
    TenacitySlides slides;
    TenacityArm arm;
    TenacityClaw doubleClaw;
    TenacityWrist wrist;

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
        chassis = new TenacityChassis(gamepad1, telemetry, hardwareMap);
        slides = new TenacitySlides(arm, gamepad1, telemetry, hardwareMap);
        arm = new TenacityArm(slides, gamepad1, telemetry, hardwareMap);
        doubleClaw = new TenacityClaw(gamepad1, telemetry, hardwareMap);
        wrist = new TenacityWrist(gamepad1, telemetry, hardwareMap);

        slides.arm = arm;
        arm.slides = slides;

        teleOpState = TeleOpState.HOME;
        scoringState = TeleOpState.FIRST_LINE;

        while (opModeInInit()){
            slides.setSlidePower();
//            arm.setArmPower();
//            doubleClaw.setClawOpen();
            wrist.setWristPosition();

            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.ManualDrive();

            arm.setArmPower();
            slides.setSlidePower();

            doubleClaw.toggleClaw();
            doubleClaw.toggleLeftClaw();
            doubleClaw.toggleRightClaw();

            wrist.setWristPosition();
            switch (teleOpState){
                case HOME:
                    arm.armState = TenacityArm.ArmState.CLOSE_INTAKE;
                    slides.slidesState = TenacitySlides.SlidesState.HOME;
                    wrist.wristPosition = TenacityWrist.WristPosition.INTAKE_POSITION;

                    if ((gamepad1.x != lastToggleX) && gamepad1.x){
                        teleOpState = scoringState;
                    }

                    if ((gamepad1.b != lastToggleB) && gamepad1.b){
                        teleOpState = TeleOpState.MANUAL;
                    }
                    break;
                case FIRST_LINE:
                    arm.armState = TenacityArm.ArmState.FIRST_LINE;
                    slides.slidesState = TenacitySlides.SlidesState.FIRST_LINE;
                    wrist.wristPosition = TenacityWrist.WristPosition.OUTTAKE_FIRST_LINE;

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
                    slides.slidesState = TenacitySlides.SlidesState.SECOND_LINE;
                    wrist.wristPosition = TenacityWrist.WristPosition.OUTTAKE_SECOND_LINE;

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
                    slides.slidesState = TenacitySlides.SlidesState.THIRD_LINE;
                    wrist.wristPosition = TenacityWrist.WristPosition.OUTTAKE_THIRD_LINE;

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
                    slides.slidesState = TenacitySlides.SlidesState.MANUAL;

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

            chassis.update();

            telemetry.addData("Pose Estimate", chassis.getPoseEstimate());
            telemetry.addData("Getting TenacityChassis Pose", chassis.getPoseVector());

            telemetry.update();
        }
    }
}