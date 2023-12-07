package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Chassis;

@TeleOp
//@Disabled
public class DriveTesting extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        PhotonCore.enable();

        Chassis chassis = new Chassis(gamepad1, telemetry, hardwareMap);
//        OdoRetract odoRetract = new OdoRetract(gamepad1, hardwareMap);
        telemetry.addLine("Waiting For Start");
        telemetry.update();

//        odoRetract.podState = OdoRetract.PodState.GROUND;
        waitForStart();

        chassis.setPoseEstimate(new Pose2d(0,0,0));
        while (opModeIsActive()){
            chassis.ManualDrive();
//            odoRetract.toggleState();
            chassis.update();

            telemetry.addData("Pose Estimate", chassis.getPoseEstimate());
            telemetry.addData("Getting Chassis Pose", chassis.getPoseVector());
            telemetry.addData("time", runtime.seconds()-previousTime);
            telemetry.update();

            previousTime = runtime.seconds();
        }
    }
}
