package org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.MecanumDriveTrain_Old;

public class RoadRunnerTest_Control {

    MecanumDriveTrain_Old drivetrain;

    Telemetry telemetry;

    public enum driveState {
        trajectoryOne,
        trajectoryTwo,
        IDLE
    }

    Trajectory trajectoryOne;
    Trajectory trajectoryTwo;

    public driveState DriveState;

    public RoadRunnerTest_Control(String flName, String frName, String brName, String blName, HardwareMap hardwareMap, Telemetry telemetry){
        drivetrain = new MecanumDriveTrain_Old(flName, frName, brName, blName, hardwareMap);

        Pose2d globalPose = new Pose2d(0,0,0);

        drivetrain.setPoseEstimate(globalPose);

        trajectoryOne = drivetrain.trajectoryBuilder(globalPose).splineToConstantHeading(new Vector2d(10, 22), 0).build();
        trajectoryTwo = drivetrain.trajectoryBuilder(trajectoryOne.end()).splineTo(new Vector2d(22, 0), -Math.toRadians(90)).build();

        DriveState = driveState.trajectoryOne;
        drivetrain.followTrajectoryAsync(trajectoryOne);

        this.telemetry = telemetry;
    }

    public void RR_Drive(){
        switch (DriveState){
            case trajectoryOne:
                if (!drivetrain.isBusy()){
                    DriveState = driveState.trajectoryTwo;
                    drivetrain.followTrajectoryAsync(trajectoryTwo);
                }

                break;
            case trajectoryTwo:
                if (!drivetrain.isBusy()){
                    DriveState = driveState.IDLE;
                }
                break;
            case IDLE:
                drivetrain.setPower(0);
        }

        drivetrain.update();
    }

    public void setTelemetry(){
        Pose2d poseEstimate = drivetrain.getPoseEstimate();

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }

}
