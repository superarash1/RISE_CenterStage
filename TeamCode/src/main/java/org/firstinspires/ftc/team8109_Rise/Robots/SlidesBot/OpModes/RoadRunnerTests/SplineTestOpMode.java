package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.RoadRunnerTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Chassis;

@Autonomous(group = "drive")
@Disabled
public class SplineTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Chassis drive = new Chassis(gamepad1, telemetry, hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(27, 9), Math.toRadians(0))
                .build();

        drive.followTrajectory(traj);

        sleep(500);
//
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
    }
}
