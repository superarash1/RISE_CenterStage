package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.TenacityArm;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.TenacitySlides;

@TeleOp
public class SlidesTesting extends LinearOpMode {

    TenacitySlides slides;
    TenacityArm arm;
    @Override
    public void runOpMode() throws InterruptedException {
        arm = new TenacityArm(slides, gamepad1, telemetry, hardwareMap);
        slides = new TenacitySlides(arm, gamepad1, telemetry, hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            slides.setSlidePower();
            slides.TuningTelemetry();

            telemetry.update();
        }
    }
}
