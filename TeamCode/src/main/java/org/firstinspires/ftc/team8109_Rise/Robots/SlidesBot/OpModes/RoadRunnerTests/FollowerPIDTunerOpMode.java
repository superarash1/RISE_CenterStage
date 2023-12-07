package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.RoadRunnerTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team8109_Rise.Resources.RoadRunnerQuickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Chassis;

@Config
@Autonomous(group = "drive")
@Disabled
public class FollowerPIDTunerOpMode extends LinearOpMode {
    public static double DISTANCE = 15; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Chassis drive = new Chassis(gamepad1, telemetry, hardwareMap);

        Pose2d startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .build();
            drive.followTrajectorySequence(trajSeq);
        }
    }
}
