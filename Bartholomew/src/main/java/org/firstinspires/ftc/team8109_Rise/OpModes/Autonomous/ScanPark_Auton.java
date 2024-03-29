package org.firstinspires.ftc.team8109_Rise.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector3D;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.Bartholomew_Chassis;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.Bartholomew_Slides;
import org.firstinspires.ftc.robotcontroller.Hardware.Sensors.Camera.OpenCV.VisionPipelines.ColorPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ScanPark_Auton extends LinearOpMode {
    OpenCvCamera camera; //TODO: Improve tracking
    ColorPipeline pipeline;

    ElapsedTime runtime = new ElapsedTime();

    int cycleCounter = 1;

    public enum ParkingStep{
        STEP_ONE,
        STEP_TWO
    }

    public enum ParkingZone{
        LEFT,
        MIDDLE,
        RIGHT
    }

    Bartholomew_Chassis bartholomewChassis;

    Bartholomew_Slides slides;
//    OdoRetract odoRetract;
    ParkingStep parkingStep;
    ParkingZone parkingZone;

    Vector3D targetPose = new Vector3D(0, 0, 0);
    double tolerance = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {
        bartholomewChassis = new Bartholomew_Chassis(gamepad1, telemetry, hardwareMap);
        slides = new Bartholomew_Slides(gamepad1, telemetry, hardwareMap);

        parkingStep = ParkingStep.STEP_ONE;

        parkingZone = ParkingZone.RIGHT;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ColorPipeline(telemetry);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened(){
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        // Adjust 0.5 left
        while (opModeInInit()){
            if (pipeline.findColor() == ColorPipeline.Colors.BLUE){
                parkingZone = ParkingZone.LEFT;
            }

            if (pipeline.findColor() == ColorPipeline.Colors.RED){
                parkingZone = ParkingZone.RIGHT;
            }

            if (pipeline.findColor() == ColorPipeline.Colors.GREEN){
                parkingZone = ParkingZone.MIDDLE;
            }

            telemetry.addData("color", pipeline.findColor());
            telemetry.addData("color", pipeline.getHue());

            telemetry.update();
        }

        while (opModeIsActive()){
            bartholomewChassis.update();
            bartholomewChassis.updatePoseEstimate();
            bartholomewChassis.goToPosePID(targetPose);

            autonLeftRed();
            slides.setSlidePower();

            Telemetry();

            telemetry.update();
        }
    }

    public void autonLeftRed(){
        slides.slidesState = Bartholomew_Slides.SlidesState.GROUND;

        switch (parkingZone){
            case LEFT:
                switch (parkingStep){
                    case STEP_ONE:
                        targetPose.set(24, 0, 0);

                        if (targetPose.findDistance(bartholomewChassis.getPoseVector()) < tolerance){
                            parkingStep = ParkingStep.STEP_TWO;
                            runtime.reset();
                        }
                        break;
                    case STEP_TWO:
                        targetPose.set(24, 24, 0);
                        break;
                }
                break;
            case MIDDLE:
                switch (parkingStep){
                    case STEP_ONE:
                        targetPose.set(24, 0, 0);

                        if (targetPose.findDistance(bartholomewChassis.getPoseVector()) < tolerance){
                            parkingStep = ParkingStep.STEP_TWO;
                            runtime.reset();
                        }
                        break;
                    case STEP_TWO:
                        targetPose.set(24, 0, 0);
                        break;
                }
                break;
            case RIGHT:
                switch (parkingStep){
                    case STEP_ONE:
                        targetPose.set(25, 0, 0);

                        if (targetPose.findDistance(bartholomewChassis.getPoseVector()) < tolerance){
                            parkingStep = ParkingStep.STEP_TWO;
                            runtime.reset();
                        }
                        break;
                    case STEP_TWO:
                        targetPose.set(25, -24, 0);
                        break;
                }
                break;
        }
    }

    public void Telemetry(){
        //TODO: show the tolerance stuff and PID values and states
        telemetry.addData("Auton State", parkingStep);
        telemetry.addData("Parking Zone", parkingZone);
        telemetry.addData("cycle count", cycleCounter);
        telemetry.addData("color", pipeline.findColor());
        telemetry.addData("color", pipeline.getHue());
        telemetry.addData("runtime", runtime.seconds());
        telemetry.addData("Pose", bartholomewChassis.getPoseEstimate());

        telemetry.addData("HeadingPID error", bartholomewChassis.HeadingPID.error);
        telemetry.addData("TranslationalX error", bartholomewChassis.TranslationalPID_X.error);
        telemetry.addData("TranslationalY error", bartholomewChassis.TranslationalPID_Y.error);
        telemetry.addData("Distance to Pose", targetPose.findDistance(bartholomewChassis.getPoseVector()));
    }
}
