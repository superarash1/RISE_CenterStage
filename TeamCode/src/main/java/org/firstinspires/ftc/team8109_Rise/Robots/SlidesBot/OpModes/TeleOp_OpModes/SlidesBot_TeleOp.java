package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.TeleOp_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Chassis;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Claw;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.OdoRetract;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.ServoIntakeArm;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.ViperSlides;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Wrist;

//@TeleOp
public class SlidesBot_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Creating objects of each mechanism
        Chassis chassis = new Chassis(gamepad1, telemetry, hardwareMap);
        ViperSlides slides = new ViperSlides(gamepad1, telemetry, hardwareMap);
        Claw claw = new Claw(gamepad1, telemetry, hardwareMap);
        Wrist wrist = new Wrist(gamepad1, hardwareMap);
        ServoIntakeArm arm = new ServoIntakeArm(gamepad1, telemetry, hardwareMap);
        OdoRetract odoRetract = new OdoRetract(gamepad1, hardwareMap);

        odoRetract.podState = OdoRetract.PodState.RETRACTED;

        telemetry.addLine("Waiting For Start");
        telemetry.update();

        while (opModeInInit()){
            odoRetract.setPodPosition();
            arm.setArmPosition();
            wrist.setPosition();
//            slides.setSlidePower();
        }

        while (opModeIsActive()){
            // Setting methods from mechanism classes to be looped
            chassis.ManualDrive();
//            chassis.chassisTelemetry();
            slides.slidesTelemetry();
            arm.setTelemetry();
            slides.toggleStates();
//
//            arm.setArmPosition();
//            arm.setTelemetry();
            if (slides.slidesState == ViperSlides.SlidesState.GROUND){
//                arm.servoPosition = ServoIntakeArm.ServoPosition.INTAKE_POSITION;
                wrist.wristPosition = Wrist.WristPosition.INTAKE_POSITION;
            } else {
//                arm.servoPosition = ServoIntakeArm.ServoPosition.OUTTAKE_POSITION;
                wrist.wristPosition = Wrist.WristPosition.OUTTAKE_POSITION;
            }


////
//            if (slides.slidesState == ViperSlides.SlidesState.LOW_JUNCTION && gamepad1.left_bumper){
//                arm.servoPosition = ServoIntakeArm.ServoPosition.DUNK_POSITION;
//            }

//            if (slides.slidesState == ViperSlides.SlidesState.JAM){
//                arm.servoPosition = ServoIntakeArm.ServoPosition.DUNK_POSITION;
//            }

            arm.slidesToggle(slides.slidesState);

//            arm.togglePosition();
//
            claw.toggleClaw();
//            claw.setPosition();
            claw.setTelemetry();

            wrist.setPosition();

            odoRetract.toggleState();

            telemetry.addData("FrontLeft", chassis.fLeft);
            telemetry.addData("FrontRight", chassis.fRight);
            telemetry.addData("BackRight", chassis.bRight);
            telemetry.addData("BackLeft", chassis.bLeft);

            // Updating telemetry to display all of the telemetry in the telemetry queue on the driver station
            telemetry.update();
        }

//
//        if (isStopRequested()){
//            odoRetract.podState = OdoRetract.PodState.GROUND;
//        }
//        //        while (odoRetract.retractionServo.getPosition() > 0.775){
//            odoRetract.setPodPosition();
//        }
    }
}
