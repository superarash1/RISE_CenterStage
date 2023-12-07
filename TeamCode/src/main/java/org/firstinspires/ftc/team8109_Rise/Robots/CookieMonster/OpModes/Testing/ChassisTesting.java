package org.firstinspires.ftc.team8109_Rise.Robots.CookieMonster.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.Robots.CookieMonster.Mechanisms.Chassis;

@TeleOp
public class ChassisTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis(gamepad1, telemetry, hardwareMap);

        telemetry.addLine("Waiting For Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            chassis.ManualDrive();
        }
    }
}
