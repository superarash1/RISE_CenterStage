package org.firstinspires.ftc.team8109_Rise.OpModes.TeleOp_Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.BeefySlides;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.BeefyChassis;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.SausageFingers;

@TeleOp
public class BeefCake_TeleOp extends LinearOpMode {
    BeefyChassis chassis;
    BeefySlides beefySlides;
    SausageFingers sausageFingers;
    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new BeefyChassis(gamepad1, telemetry, hardwareMap);
        beefySlides = new BeefySlides(gamepad1, telemetry, hardwareMap);
        sausageFingers = new SausageFingers(gamepad1, telemetry, hardwareMap);

        while (opModeInInit()){
            beefySlides.setSlidePower();
            sausageFingers.setClawOpen();

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
            telemetry.addData("Getting BeefyChassis Pose", chassis.getPoseVector());

            telemetry.update();
        }
    }
}