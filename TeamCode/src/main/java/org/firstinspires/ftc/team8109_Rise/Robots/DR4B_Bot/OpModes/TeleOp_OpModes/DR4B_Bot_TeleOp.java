package org.firstinspires.ftc.team8109_Rise.Robots.DR4B_Bot.OpModes.TeleOp_OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.Robots.DR4B_Bot.Chassis;
import org.firstinspires.ftc.team8109_Rise.Robots.DR4B_Bot.DR4B;

@TeleOp
@Disabled
public class DR4B_Bot_TeleOp extends LinearOpMode {
    Chassis chassis;
    DR4B dr4b;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new Chassis(gamepad1, telemetry, hardwareMap);
        dr4b = new DR4B(gamepad1, telemetry, hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            chassis.ManualDrive();
            dr4b.Set_DR4B_Power();
            dr4b.DriverControl();
            dr4b.Claw();
            dr4b.Telemetry();
//            telemetry.addData("Bot Angle", chassis.imu.Angle_FieldCentric());
//            telemetry.update();
        }
    }
}
