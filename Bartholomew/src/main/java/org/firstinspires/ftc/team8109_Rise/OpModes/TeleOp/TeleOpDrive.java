package org.firstinspires.ftc.team8109_Rise.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.Bartholomew_Chassis;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.Bartholomew_Claw;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.Bartholomew_Slides;

public class TeleOpDrive extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bartholomew_Chassis chassis = new Bartholomew_Chassis(gamepad1, telemetry, hardwareMap);
        Bartholomew_Slides slides = new Bartholomew_Slides(gamepad1, telemetry, hardwareMap);
        Bartholomew_Claw claw = new Bartholomew_Claw(gamepad1, telemetry, hardwareMap);
        telemetry.addLine("Waiting For Start");
        telemetry.update();

        waitForStart();

        chassis.setPoseEstimate(new Pose2d(0,0,0));
        while (opModeIsActive()){
            chassis.ManualDrive();
            chassis.update();

            slides.toggleStates();

            claw.toggleClaw();

            telemetry.addData("Pose Estimate", chassis.getPoseEstimate());
            telemetry.addData("Getting Bartholomew_Chassis Pose", chassis.getPoseVector());
            telemetry.addData("time", runtime.seconds()-previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
