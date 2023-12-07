package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Claw;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.ServoIntakeArm;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.ViperSlides;

//@TeleOp
public class SlidesConeStackTesting extends LinearOpMode {
    ViperSlides slides;
    ServoIntakeArm arm;
    Claw claw;
    @Override
    public void runOpMode() throws InterruptedException {

        slides = new ViperSlides(gamepad1, telemetry, hardwareMap);
        arm = new ServoIntakeArm(gamepad1, telemetry, hardwareMap);
        claw = new Claw(gamepad1, telemetry, hardwareMap);
        telemetry.addLine("Waiting For Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            slides.autonLevelsTesting();
            arm.setArmPosition();
            claw.toggleClaw();

            Telemetry();
            telemetry.update();
        }
    }

    public void Telemetry(){
        telemetry.addData("Level", slides.slidesState);
        telemetry.addData("Height", slides.getHeight());
        telemetry.addData("SlidePID Error", slides.slidesPID.error);
        telemetry.addData("SlidePID Proportion", slides.slidesPID.P);
        telemetry.addData("SlidePID Integral", slides.slidesPID.I);
        telemetry.addData("SlidePID Derivative", slides.slidesPID.D);
    }
}
