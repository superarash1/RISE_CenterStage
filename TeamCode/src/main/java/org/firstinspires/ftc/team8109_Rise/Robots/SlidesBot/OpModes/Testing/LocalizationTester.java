package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Chassis;

//@TeleOp
public class LocalizationTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Chassis drive = new Chassis(gamepad1, telemetry, hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.DPad_Drive();
            drive.update();

            //
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", drive.angleWrap(poseEstimate.getHeading()));
            telemetry.addData("Pod Positions", drive.odometry.getWheelPositions());
            telemetry.update();
        }
    }
}
