package org.firstinspires.ftc.team8109_Rise.Robots.CookieMonster.OpModes.TeleOp_Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.Robots.CookieMonster.Mechanisms.Arm;
import org.firstinspires.ftc.team8109_Rise.Robots.CookieMonster.Mechanisms.Chassis;
import org.firstinspires.ftc.team8109_Rise.Robots.CookieMonster.Mechanisms.Gate;
import org.firstinspires.ftc.team8109_Rise.Robots.CookieMonster.Mechanisms.Optake;

@TeleOp
@Disabled
public class CookieMonster_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis(gamepad1, telemetry, hardwareMap);
        Gate gate = new Gate(gamepad1, telemetry, hardwareMap);
        Optake optake = new Optake(gamepad1, telemetry, hardwareMap);
        Arm arm = new Arm(gamepad1, telemetry, hardwareMap);

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