package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector3D;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.TenacityChassis;

//@Autonomous
public class TrapezoidalMotionProfileTuner extends LinearOpMode {

    public static double DISTANCE = 30; // in

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private TenacityChassis drive;

    enum Mode {
        FORWARDS,
        BACKWARDS
    }

    double translationalTolerance = 0.25;
    double headingTolerance = 0.01;
    private Mode mode;

    Vector3D targetPose;

    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: Check strafe axis and if motion works on on multiple dimensions

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new TenacityChassis(gamepad1, telemetry, hardwareMap);

        mode = Mode.FORWARDS;

        targetPose = new Vector3D(0, 0, 0);

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        while (opModeInInit()){
            telemetry.addData("max_velocity", drive.TranslationalProfile_X.max_velocity);
            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) {
            drive.goToPoseTrapezoidal(targetPose);
            switch (mode) {
                case FORWARDS:
//                    if (Math.abs(drive.trapezoidalTranslationalError) < translationalTolerance && drive.HeadingPID.error < headingTolerance) {
//                        mode = Mode.BACKWARDS;
//                    }

                    targetPose.set(DISTANCE, 0, 0);
                    // calculate and set the motor power
                    // update telemetry
                    break;
                case BACKWARDS:
                    if (Math.abs(drive.trapezoidalTranslationalError) < translationalTolerance && drive.HeadingPID.error < headingTolerance) {
                        mode = Mode.FORWARDS;
                    }

                    targetPose.set(-DISTANCE, 0, 0);
                    break;
            }

            Telemetry();
            telemetry.update();
        }
    }

    public void Telemetry(){
        telemetry.addData("mode", mode);
        telemetry.addData("translational error", drive.trapezoidalTranslationalError);
        telemetry.addData("heading error", drive.HeadingPID.error);
        telemetry.addData("X Profile error", drive.TranslationalProfile_X.positionError);
        telemetry.addData("accel_dt_test", drive.TranslationalProfile_X.acceleration_dt_test);

        telemetry.addData("ProfileState", drive.TranslationalProfile_X.ProfileState);
        telemetry.addData("current_dt", drive.TranslationalProfile_X.current_dt);
        telemetry.addData("halfway_distance", drive.TranslationalProfile_X.halfway_distance);
        telemetry.addData("acceleration_dt", drive.TranslationalProfile_X.acceleration_dt);
        telemetry.addData("accel dt written out", Math.sqrt(Math.abs(drive.TranslationalProfile_X.halfway_distance) / (0.5 * drive.TranslationalProfile_X.max_acceleration)));
        telemetry.addData("Written Out", drive.TranslationalProfile_X.max_acceleration * drive.TranslationalProfile_X.acceleration_dt);
        telemetry.addData("ideal accel dt", drive.TranslationalProfile_X.max_velocity / drive.TranslationalProfile_X.max_acceleration);
        telemetry.addData("max velocity", drive.TranslationalProfile_X.max_velocity);
        telemetry.addData("max accel", drive.TranslationalProfile_X.max_acceleration);
        telemetry.addData("accel distance", drive.TranslationalProfile_X.acceleration_distance);
    }
}
