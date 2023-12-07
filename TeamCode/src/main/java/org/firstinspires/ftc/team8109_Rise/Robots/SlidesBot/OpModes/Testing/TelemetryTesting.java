package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@TeleOp
@Config
public class TelemetryTesting extends LinearOpMode {

    static double test = 10;
    double test2 = 10;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("hi", test2);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("hi again :)", test2);
            telemetry.update();
        }
    }
}
