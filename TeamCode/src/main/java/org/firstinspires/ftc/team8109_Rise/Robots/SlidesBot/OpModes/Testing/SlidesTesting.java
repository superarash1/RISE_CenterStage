package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Claw;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.ServoIntakeArm;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.ViperSlides;

//@TeleOp
public class SlidesTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ViperSlides slides = new ViperSlides(gamepad1, telemetry, hardwareMap);
        ServoIntakeArm arm = new ServoIntakeArm(gamepad1, telemetry, hardwareMap);
        Claw claw = new Claw(gamepad1, telemetry, hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            slides.toggleStates();
//            slides.setSlidePower();
            slides.slidesTelemetry();
//            claw.toggleClaw();

            arm.setArmPosition();
            arm.setTelemetry();

            telemetry.update();
        }
    }
}
