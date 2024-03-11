package org.firstinspires.ftc.team8109_Rise.OpModes.TeleOp_Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.CookieMonster_Arm;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.CookieMonster_Chassis;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.CookieMonster_Gate;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.CookieMonster_Optake;

@TeleOp
@Disabled
public class CookieMonster_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CookieMonster_Chassis chassis = new CookieMonster_Chassis(gamepad1, telemetry, hardwareMap);
        CookieMonster_Gate gate = new CookieMonster_Gate(gamepad1, telemetry, hardwareMap);
        CookieMonster_Optake optake = new CookieMonster_Optake(gamepad1, telemetry, hardwareMap);
        CookieMonster_Arm arm = new CookieMonster_Arm(gamepad1, telemetry, hardwareMap);

        telemetry.addLine("Waiting For Start");

        while (opModeInInit()){
            arm.SetArmPower();
            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.ManualDrive();
            gate.toggleGate();
            optake.toggleIntake();
            arm.DriverControl();

            chassis.Telemetry();
            arm.Telemetry();
            telemetry.update();
        }
    }
}