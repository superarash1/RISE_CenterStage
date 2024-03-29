package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.TenacitySlides;

//@TeleOp
public class SlidesTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TenacitySlides slides = new TenacitySlides(gamepad1, telemetry, hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            slides.setSlidePower();
            slides.slidesTelemetry();

            telemetry.update();
        }
    }
}
