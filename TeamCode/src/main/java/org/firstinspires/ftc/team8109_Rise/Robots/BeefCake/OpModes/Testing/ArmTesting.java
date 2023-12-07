package org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Mechanisms.Arm;

@TeleOp
public class ArmTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(gamepad1, telemetry, hardwareMap);

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