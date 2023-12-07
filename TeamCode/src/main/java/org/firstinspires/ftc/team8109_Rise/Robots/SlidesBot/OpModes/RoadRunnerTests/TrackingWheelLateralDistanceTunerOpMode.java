package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.RoadRunnerTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.team8109_Rise.Resources.RoadRunnerQuickstart.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Chassis;
import org.firstinspires.ftc.team8109_Rise.Sensors.Odometry.StandardThreeWheelOdoLocalizer;

@Autonomous
public class TrackingWheelLateralDistanceTunerOpMode extends LinearOpMode {
    public static int NUM_TURNS = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        Chassis drive = new Chassis(gamepad1, telemetry, hardwareMap);

        if (!(drive.getLocalizer() instanceof StandardThreeWheelOdoLocalizer)) {
            RobotLog.setGlobalErrorMsg("StandardTrackingWheelLocalizer is not being set in the "
                    + "drive class. Ensure that \"setLocalizer(new StandardTrackingWheelLocalizer"
                    + "(hardwareMap));\" is called in SampleMecanumDrive.java");
        }

        telemetry.addLine("Prior to beginning the routine, please read the directions "
                + "located in the comments of the opmode file.");
        telemetry.addLine("Press play to begin the tuning routine.");
        telemetry.addLine("");
        telemetry.addLine("Press Y/△ to stop the routine.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.update();

        double headingAccumulator = 0;
        double lastHeading = 0;

        boolean tuningFinished = false;

        while (!isStopRequested() && !tuningFinished) {
            Pose2d vel = new Pose2d(0, 0, -gamepad1.right_stick_x);
            drive.setDrivePower(vel);

            drive.update();

            double heading = drive.getPoseEstimate().getHeading();
            double deltaHeading = heading - lastHeading;

            headingAccumulator += Angle.normDelta(deltaHeading);
            lastHeading = heading;

            telemetry.clearAll();
            telemetry.addLine("Total Heading (deg): " + Math.toDegrees(headingAccumulator));
            telemetry.addLine("Raw Heading (deg): " + Math.toDegrees(heading));
            telemetry.addLine();
            telemetry.addLine("Press Y/△ to conclude routine");
            telemetry.update();

            if (gamepad1.y)
                tuningFinished = true;
        }

        telemetry.clearAll();
        telemetry.addLine("Localizer's total heading: " + Math.toDegrees(headingAccumulator) + "°");
        telemetry.addLine("Effective LATERAL_DISTANCE: " +
                (headingAccumulator / (NUM_TURNS * Math.PI * 2)) * StandardTrackingWheelLocalizer.LATERAL_DISTANCE);

        telemetry.update();

        while (!isStopRequested()) idle();
    }
}
