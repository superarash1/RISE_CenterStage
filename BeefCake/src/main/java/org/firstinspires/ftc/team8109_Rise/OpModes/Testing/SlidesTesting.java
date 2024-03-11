package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.BeefySlides;

//@TeleOp
public class SlidesTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BeefySlides slides = new BeefySlides(gamepad1, telemetry, hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            slides.toggleStates();
            slides.slidesTelemetry();

            telemetry.update();
        }
    }
}
