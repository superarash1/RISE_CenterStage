package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector3D;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.BeefyChassis;

@Autonomous
public class PID_PointFollowerTest extends LinearOpMode {

    BeefyChassis chassis;
    Vector3D point = new Vector3D(0, 0, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new BeefyChassis(gamepad1, telemetry, hardwareMap);

        chassis.TranslationalPID_X.setPIDCoefficients(0.13, 0.0375, 0, 0.001);//0.03 0.001
        chassis.TranslationalPID_Y.setPIDCoefficients(0.13, 0.0375, 0, 0.001);
        chassis.HeadingPID.setPIDCoefficients(1.5, 0.04, 0, 0);

        waitForStart();

        while (opModeIsActive()){
            chassis.update();
            chassis.updatePoseEstimate();
            chassis.goToPosePID(point);

            telemetry.addData("Pose", chassis.getPoseEstimate());

            telemetry.addData("HeadingPID error", chassis.HeadingPID.error);
            telemetry.addData("TranslationalX error", chassis.TranslationalPID_X.error);
            telemetry.addData("TranslationalY error", chassis.TranslationalPID_Y.error);
            telemetry.addData("Distance to Pose", point.findDistance(chassis.getPoseVector()));

            telemetry.addData("X PID Proportion", chassis.TranslationalPID_X.P);
            telemetry.addData("Y PID Proportion", chassis.TranslationalPID_Y.P);
            telemetry.addData("Heading PID Proportion", chassis.HeadingPID.P);

            telemetry.addData("TranslationalX PID Derivative", chassis.TranslationalPID_X.D);
            telemetry.addData("TranslationalY PID Derivative", chassis.TranslationalPID_Y.D);

            telemetry.addData("X PID Integral", chassis.TranslationalPID_X.I);
            telemetry.addData("Y PID Integral", chassis.TranslationalPID_Y.I);
            telemetry.addData("Heading PID Integral", chassis.HeadingPID.I);

            telemetry.update();
        }
    }
}
