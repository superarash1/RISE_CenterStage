package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.CookieMonster_Chassis;

//@TeleOp
public class ChassisTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CookieMonster_Chassis chassis = new CookieMonster_Chassis(gamepad1, telemetry, hardwareMap);

        telemetry.addLine("Waiting For Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            chassis.ManualDrive();
        }
    }
}
