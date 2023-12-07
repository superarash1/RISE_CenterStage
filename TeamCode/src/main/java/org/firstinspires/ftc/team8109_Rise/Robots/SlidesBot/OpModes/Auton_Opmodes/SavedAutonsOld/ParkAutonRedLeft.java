package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.Auton_Opmodes.SavedAutonsOld;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team8109_Rise.Hardware.Intakes.ServoClaw;
import org.firstinspires.ftc.team8109_Rise.Math.Vectors.Vector3D;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Chassis;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Claw;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.OdoRetract;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.ServoIntakeArm;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.ViperSlides;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Wrist;
//import org.firstinspires.ftc.team8109_Rise.Sensors.Camera.OpenCV.VisionPipelines.ColorPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ParkAutonRedLeft extends LinearOpMode {

    OpenCvCamera camera; //TODO: Improve tracking
//    ColorPipeline pipeline;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime globalTime = new ElapsedTime();

//    static Signal_Identifier pipeline;

    int cycleCounter = 1;

    public enum AutonState {
        PUSH_CONE,
        RETURN_TO_POLE,
        GO_TO_SCORE_PRELOAD,
        SCORE_PRELOAD,
        CYCLE,
        PARK
    }

    public enum CycleState{
        TO_CONE_STACK,
        PICK_UP_CONE,
        TO_HIGH_JUNCTION,
        SCORE_CONE
    }

    public enum ParkingStep{
        STEP_ONE,
        STEP_TWO
    }

    public enum ParkingZone{
        LEFT,
        MIDDLE,
        RIGHT
    }


    AutonState autonState;
    CycleState cycleState;
    ParkingStep parkingStep;
    ParkingZone parkingZone;

    Chassis chassis;
    ViperSlides slides;
    ServoIntakeArm arm;
    Wrist wrist;
    Claw claw;
    OdoRetract odoRetract;

    Vector3D targetPose = new Vector3D(0, 0, 0);
    double tolerance = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new Chassis(gamepad1, telemetry, hardwareMap);
        slides = new ViperSlides(gamepad1, telemetry, hardwareMap);
        arm = new ServoIntakeArm(gamepad1,telemetry, hardwareMap);
        wrist = new Wrist(gamepad1, hardwareMap);
        claw = new Claw(gamepad1, telemetry, hardwareMap);
        odoRetract = new OdoRetract(gamepad1, hardwareMap);

        claw.clawState = ServoClaw.ClawState.CLOSED;
        autonState = AutonState.PARK;
        parkingStep = ParkingStep.STEP_ONE;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        pipeline = new ColorPipeline(telemetry);
//
//        camera.setPipeline(pipeline);
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

        while (opModeInInit()){
            claw.setPosition();
            odoRetract.podState = OdoRetract.PodState.GROUND;

//            if (pipeline.findColor() == ColorPipeline.Colors.BLUE){
//                parkingZone = ParkingZone.LEFT;
//            }
//
//            if (pipeline.findColor() == ColorPipeline.Colors.RED){
//                parkingZone = ParkingZone.RIGHT;
//            }
//
//            if (pipeline.findColor() == ColorPipeline.Colors.GREEN){
//                parkingZone = ParkingZone.MIDDLE;
//            }
//
//            telemetry.addData("color", pipeline.findColor());
//            telemetry.addData("color", pipeline.getHue());

            telemetry.update();
        }

//        parkingZone = ParkingZone.MIDDLE;

        while (opModeIsActive()){
            chassis.update();
            chassis.updatePoseEstimate();

            chassis.goToPosePID(targetPose);

            autonLeftRed();

            slides.setSlidePower();
            claw.setPosition();
            arm.setArmPosition();
            wrist.setPosition();
            odoRetract.setPodPosition();

            Telemetry();
            telemetry.update();
        }
    }

    public void autonLeftRed(){
        odoRetract.podState = OdoRetract.PodState.RETRACTED;
        slides.slidesState = ViperSlides.SlidesState.GROUND;
        arm.servoPosition = ServoIntakeArm.ServoPosition.INTAKE_POSITION;
        wrist.wristPosition = Wrist.WristPosition.INTAKE_POSITION;
        claw.clawState = ServoClaw.ClawState.OPEN;

        switch (parkingZone){
            case LEFT:
                switch (parkingStep){
                    case STEP_ONE:
                        targetPose.set(26, 0, 0);

                        if (targetPose.findDistance(chassis.getPoseVector()) < tolerance){
                            parkingStep = ParkingStep.STEP_TWO;
                            runtime.reset();
                        }
                        break;
                    case STEP_TWO:
                        targetPose.set(26, 10, 0);
                        break;
                }
                break;
            case MIDDLE:
                switch (parkingStep){
                    case STEP_ONE:
                        targetPose.set(26, 0, 0);

                        if (targetPose.findDistance(chassis.getPoseVector()) < tolerance){
                            parkingStep = ParkingStep.STEP_TWO;
                            runtime.reset();
                        }
                        break;
                    case STEP_TWO:
                        targetPose.set(26, 0, 0);
                        break;
                }
                break;
            case RIGHT:
                switch (parkingStep){
                    case STEP_ONE:
                        targetPose.set(26, 0, 0);

                        if (targetPose.findDistance(chassis.getPoseVector()) < tolerance){
                            parkingStep = ParkingStep.STEP_TWO;
                            runtime.reset();
                        }
                        break;
                    case STEP_TWO:
                        targetPose.set(26, -10, 0);
                        break;
                }
                break;
        }
    }

    public void Telemetry(){
        //TODO: show the tolerance stuff and PID values and states
        telemetry.addData("Auton State", parkingStep);
        telemetry.addData("Parking Zone", parkingZone);
//        telemetry.addData("cycle count", cycleCounter);
//        telemetry.addData("color", pipeline.findColor());
//        telemetry.addData("color", pipeline.getHue());
//        telemetry.addData("runtime", runtime.seconds());
//        telemetry.addData("claw position", claw.getPositionDegrees());

//        telemetry.addData("HeadingPID Proportion",chassis.HeadingPID.P);
//        telemetry.addData("HeadingPID error",chassis.HeadingPID.error);
//        telemetry.addData("TranslationalX Proportion",chassis.TranslationalPID_X.P);
//        telemetry.addData("TranslationalX error",chassis.TranslationalPID_X.error);
//        telemetry.addData("TranslationalY Proportion",chassis.TranslationalPID_Y.P);
//        telemetry.addData("TranslationalY error",chassis.TranslationalPID_Y.error);
        telemetry.addData("Distance to Pose", targetPose.findDistance(chassis.getPoseVector()));
    }
}
