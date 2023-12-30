package org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.OpModes.TeleOp_Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Mechanisms.Chassis;

@TeleOp
public class BeefCake_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis(gamepad1, telemetry, hardwareMap);

        while (opModeInInit()){
            telemetry.addLine("Waiting For Start");
            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.ManualDrive();

            chassis.chassisTelemetry();

            chassis.update();

            telemetry.addData("Pose Estimate", chassis.getPoseEstimate());
            telemetry.addData("Getting Chassis Pose", chassis.getPoseVector());
            telemetry.update();

            telemetry.update();
        }
    }
}