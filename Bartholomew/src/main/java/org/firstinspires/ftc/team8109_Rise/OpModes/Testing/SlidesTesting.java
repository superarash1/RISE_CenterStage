package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.Bartholomew_Slides;

//@TeleOp
public class SlidesTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Bartholomew_Slides slides = new Bartholomew_Slides(gamepad1, telemetry, hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            slides.toggleStates();
            slides.slidesTelemetry();

            telemetry.update();
        }
    }
}
