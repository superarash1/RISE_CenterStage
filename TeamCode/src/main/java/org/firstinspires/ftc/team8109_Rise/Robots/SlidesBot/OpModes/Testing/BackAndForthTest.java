package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team8109_Rise.Math.Vectors.Vector3D;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Chassis;

//@Autonomous
public class BackAndForthTest extends LinearOpMode {

    Chassis chassis;
    Vector3D point = new Vector3D(20, 0, 0);
    Vector3D point2 = new Vector3D(-20, 0, 0);

    // Forwards, left, and clockwise

    enum DriveState{
        FORWARDS,
        BACKWARDS
    }

    DriveState driveState;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new Chassis(gamepad1, telemetry, hardwareMap);
        waitForStart();

        driveState = DriveState.FORWARDS;
        while (opModeIsActive()){
            chassis.update();
            chassis.updatePoseEstimate();

            switch (driveState){
                case FORWARDS:
                    chassis.goToPosePID(point);
                    break;
                case BACKWARDS:
                    chassis.goToPosePID(point2);
            }
            chassis.goToPosePID(point);

            telemetry.addData("Pose", chassis.getPoseEstimate());

            telemetry.addData("HeadingPID error", chassis.HeadingPID.error);
            telemetry.addData("TranslationalX error", chassis.TranslationalPID_X.error);
            telemetry.addData("TranslationalY error", chassis.TranslationalPID_Y.error);
            telemetry.addData("Distance to Pose", point.findDistance(chassis.getPoseVector()));

//            telemetry.addData("X PID Proportion", chassis.TranslationalPID_X.P);
//            telemetry.addData("Y PID Proportion", chassis.TranslationalPID_Y.P);
//            telemetry.addData("Heading PID Proportion", chassis.HeadingPID.P);

            telemetry.addData("TranslationalX PID Derivative", chassis.TranslationalPID_X.D);
            telemetry.addData("TranslationalY PID Derivative", chassis.TranslationalPID_Y.D);

            telemetry.update();
        }
    }
}
