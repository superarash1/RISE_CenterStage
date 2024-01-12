package org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.OpModes.TeleOp_Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Mechanisms.BeefySlides;
import org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Mechanisms.Chassis;
import org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Mechanisms.SausageFingers;

@TeleOp
public class BeefCake_TeleOp extends LinearOpMode {
    Chassis chassis;
    BeefySlides beefySlides;
    SausageFingers sausageFingers;
    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new Chassis(gamepad1, telemetry, hardwareMap);
        beefySlides = new BeefySlides(gamepad1, telemetry, hardwareMap);
        sausageFingers = new SausageFingers(gamepad1, telemetry, hardwareMap);

        while (opModeInInit()){
            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.ManualDrive();
            beefySlides.toggleStates();
            sausageFingers.toggleClaw();

            chassis.chassisTelemetry();

            chassis.update();

            telemetry.addData("Pose Estimate", chassis.getPoseEstimate());
            telemetry.addData("Getting Chassis Pose", chassis.getPoseVector());

            telemetry.update();
        }
    }
}