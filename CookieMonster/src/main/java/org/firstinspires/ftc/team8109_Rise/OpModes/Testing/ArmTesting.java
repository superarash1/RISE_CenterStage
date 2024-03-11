package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.CookieMonster_Arm;

//@TeleOp
public class ArmTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CookieMonster_Arm arm = new CookieMonster_Arm(gamepad1, telemetry, hardwareMap);

        while (opModeInInit()){
            arm.SetArmPower();
            arm.Telemetry();
            telemetry.update();
        }

        while (opModeIsActive()){
            arm.DriverControl();
            arm.Telemetry();
            telemetry.update();
        }
    }
}