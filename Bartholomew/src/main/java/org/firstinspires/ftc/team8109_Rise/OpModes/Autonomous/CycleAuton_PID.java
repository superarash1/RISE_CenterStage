package org.firstinspires.ftc.team8109_Rise.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.Hardware.Intakes.ServoClaw;
import org.firstinspires.ftc.robotcontroller.Hardware.Sensors.Camera.OpenCV.VisionPipelines.ColorPipeline;
import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector3D;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.Bartholomew_Chassis;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.Bartholomew_Claw;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.Bartholomew_Slides;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Autonomous
public class CycleAuton_PID extends LinearOpMode {

    OpenCvCamera camera; //TODO: Improve tracking
    static ColorPipeline pipeline;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime globalTime = new ElapsedTime();
    int cycleCounter = 1;

    public enum AutonState {
        PUSH_CONE,
        RETURN_TO_POLE,
        GO_TO_SCORE_PRELOAD,
        DUNK_CONE,
        SCORE_PRELOAD,
        CYCLE,
        PARK
    }

    public enum CycleState{
        LINE_UP,
        WAIT_ONE,
        TO_CONE_STACK,
        PICK_UP_CONE,
        BOOST_UP,
        MOVE_OUT,
        WAIT_TWO,
        TO_HIGH_JUNCTION,
        WAIT_THREE,
        DUNK_CONE,
        SCORE_CONE
    }

    public enum ParkingStep{
        STEP_ONE,
        STEP_TWO,
        STEP_THREE
    }

    public enum ParkingZone{
        LEFT,
        MIDDLE,
        RIGHT
    }

    AutonState autonState;
    public CycleState cycleState;
    ParkingStep parkingStep;
    ParkingZone parkingZone;

    double currentTime = 0;
    ElapsedTime loopTimes = new ElapsedTime();

    double lastTime;

    Bartholomew_Chassis chassis;
    Bartholomew_Slides slides;
    Bartholomew_Claw claw;

    boolean stop = false;
    Vector3D targetPose = new Vector3D(0, 0, 0);
    double translationalTolerance = 0.9;
    double headingTolerance = 0.04;

    @Override
    public void runOpMode() throws InterruptedException {

        autonState = AutonState.PUSH_CONE;
        cycleState = CycleState.TO_CONE_STACK;
        parkingStep = ParkingStep.STEP_THREE;

        chassis = new Bartholomew_Chassis(gamepad1, telemetry, hardwareMap);
        slides = new Bartholomew_Slides(gamepad1, telemetry, hardwareMap);
        claw = new Bartholomew_Claw(gamepad1, telemetry, hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ColorPipeline(telemetry);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        chassis.setPoseEstimate(new Pose2d(0,0,0));

        while (opModeInInit()){
            slides.setSlidePower();
            claw.setPosition();

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

        globalTime.reset();
        runtime.reset();

        loopTimes.reset();
        lastTime = loopTimes.milliseconds();

        while (opModeIsActive()){
            chassis.update();
            chassis.updatePoseEstimate();

            autonLeftRed();

            slides.setSlidePower();
            claw.setPosition();

            currentTime = loopTimes.milliseconds();
            Telemetry();
            telemetry.update();
            lastTime = currentTime;
        }
    }

    //TODO: odo retract can go with bartholomewChassis control


    //TODO: move all positions a bit back
    public void autonLeftRed(){
        switch (autonState){
            case PUSH_CONE:
                chassis.TranslationalPID_X.setPIDCoefficients(0.15, 0.01, 0, 0.001);
                chassis.TranslationalPID_Y.setPIDCoefficients(0.15, 0.01, 0, 0.001);
                chassis.HeadingPID.setPIDCoefficients(1.5, 0.02, 0, 0); // 0.03 0.1 0.001
                targetPose.set(-3, 48, Math.toRadians(2));

                chassis.goToPosePID(targetPose);

                claw.clawState = ServoClaw.ClawState.CLOSED;


                if (withinPoseTolerance()){
                    autonState = AutonState.GO_TO_SCORE_PRELOAD;

                    chassis.TranslationalPID_X.setPIDCoefficients(0.35, 0.0375, 0, 0.0025);//0.03 0.001
                    chassis.TranslationalPID_Y.setPIDCoefficients(0.4, 0.0375, 0, 0.01);
                    chassis.HeadingPID.setPIDCoefficients(2, 0.02, 0, 0.001);
                    runtime.reset();
                }
                break;
            case GO_TO_SCORE_PRELOAD:
                translationalTolerance = 0.2;
                slides.slidesState = Bartholomew_Slides.SlidesState.HIGH_JUNCTION;

                targetPose.set(3, 56.2, 0.9531);
                chassis.goToPosePID(targetPose);

                if (withinPoseTolerance() && (runtime.seconds() > 0.75)){
                    autonState = AutonState.SCORE_PRELOAD;
                    runtime.reset();
                }
                break;
            case SCORE_PRELOAD:
                claw.clawState = ServoClaw.ClawState.OPEN;
                chassis.goToPosePID(targetPose);

                if ((claw.getPositionDegrees() > 50) && runtime.seconds() > 0.1){
                    autonState = AutonState.CYCLE;
                    runtime.reset();
                    translationalTolerance = 1;
                }

                if ((claw.getPositionDegrees() > 140) && (Math.abs(slides.slidesPID.error) < slides.slidesPID.tolerance)) {
                    autonState = AutonState.CYCLE;
                    runtime.reset();
                }
                break;
            case CYCLE:
                switch (cycleState){
                    case TO_CONE_STACK:
                        translationalTolerance = 0.25;
                        chassis.TranslationalPID_X.setPIDCoefficients(0.25, 0.0375, 0, 0.006);//0.03 0.001
                        chassis.TranslationalPID_Y.setPIDCoefficients(0.25, 0.0375, 0, 0.0015);
                        chassis.HeadingPID.setPIDCoefficients(2.5, 0.03, 0, 0.0015);
                        targetPose.set(-25.35, 48.65, 0.14); //-25.4, 48.65, 0.14

                        chassis.goToPosePID(targetPose);
                        switch (cycleCounter){
                            case 1:
                                slides.slidesState = Bartholomew_Slides.SlidesState.CONESTACK_FIVE;
                                break;
                            case 2:
                                slides.slidesState = Bartholomew_Slides.SlidesState.CONESTACK_FOUR;
                                break;
                            case 3:
                                slides.slidesState = Bartholomew_Slides.SlidesState.CONESTACK_THREE;
                                break;
                            case 4:
                                slides.slidesState = Bartholomew_Slides.SlidesState.CONESTACK_TWO;
                                break;
                            case 5:
                                slides.slidesState = Bartholomew_Slides.SlidesState.GROUND;
                                break;
                        }

                        if (withinPoseTolerance()){
                            cycleState = CycleState.PICK_UP_CONE;
                            runtime.reset();
                            translationalTolerance = 0.9;
                        }
                        break;

                    case PICK_UP_CONE:
                        claw.clawState = ServoClaw.ClawState.CLOSED;
                        if ((claw.getPositionDegrees() < 110) && runtime.seconds() > 0.5) {
                            cycleState = CycleState.BOOST_UP;
                            runtime.reset();
                        }
                        break;

                    case BOOST_UP:
                        slides.slidesState = Bartholomew_Slides.SlidesState.MIDDLE_JUNCTION;
                        claw.clawState = ServoClaw.ClawState.CLOSED;

                        if (runtime.seconds() > 0.25) {
                            chassis.TranslationalPID_X.setPIDCoefficients(0.13, 0.0375, 0, 0.001);//0.03 0.001
                            chassis.TranslationalPID_Y.setPIDCoefficients(0.13, 0.0375, 0, 0.001);
                            chassis.HeadingPID.setPIDCoefficients(1.5, 0.03, 0, 0.001);

                            cycleState = CycleState.MOVE_OUT;
                            slides.slidesState = Bartholomew_Slides.SlidesState.HIGH_JUNCTION;

                            runtime.reset();
                        }
                        break;
                    case MOVE_OUT:
                        chassis.setPower(-0.5);

                        if (runtime.seconds() > 1){
                            cycleState = CycleState.TO_HIGH_JUNCTION;
                            runtime.reset();
                        }

                        break;

                    case TO_HIGH_JUNCTION:
                        targetPose.set(3, 56.2, 0.9531); //2.175, 56.2, 0.9531
                        chassis.goToPosePID(targetPose);
                        if (withinPoseTolerance()){
                            cycleState = CycleState.SCORE_CONE;
                            runtime.reset();
                        }
                        break;
                    case SCORE_CONE:
                        if ((claw.getPositionDegrees() > 140) && runtime.seconds() > 0.2){
                            cycleState = CycleState.TO_CONE_STACK;
                            cycleCounter++;
                            runtime.reset();
                        }
                        break;
                }
                if ((globalTime.seconds() > 27) || cycleCounter > 5){
                    autonState = AutonState.PARK;
                }
                break;
            case PARK:
                slides.slidesState = Bartholomew_Slides.SlidesState.GROUND;
                claw.clawState = ServoClaw.ClawState.OPEN;

                chassis.goToPosePID(targetPose);
                switch (parkingStep){
                    case STEP_ONE:
                        targetPose.set(45, 0, 0);

                        if (withinPoseTolerance()){
                            parkingStep = ParkingStep.STEP_TWO;
                            runtime.reset();
                        }
                        break;
                    case STEP_TWO:
                        targetPose.set(28.335, 0, 0);
                        if (withinPoseTolerance()){
                            parkingStep = ParkingStep.STEP_THREE;
                            runtime.reset();
                        }
                        break;
                    case STEP_THREE:
                        switch (parkingZone){
                            case LEFT:
                                targetPose.set(-24, 50.5, Math.toRadians(101));
                                break;
                            case MIDDLE:
                                targetPose.set(0, 51, Math.toRadians(101));

                                break;
                            case RIGHT:
                                targetPose.set(13, 51, Math.toRadians(101));
                                break;
                        }
                        //TODO: yeet slides down at end of auton

                        if (withinPoseTolerance()){
                            break;
                        }
                }
        }
    }

    public void Telemetry(){
        //TODO: show the tolerance stuff and PID values and states
        telemetry.addData("Auton State", autonState);
        telemetry.addData("Cycle State", cycleState);
        telemetry.addData("cycle count", cycleCounter);
        telemetry.addData("Parking Zone", parkingZone);
        telemetry.addData("Slides error", slides.slidesPID.error);

        telemetry.addData("Pose Estimate", chassis.getPoseEstimate());
        telemetry.addData("Getting Bartholomew_Chassis Pose", chassis.getPoseVector());

        telemetry.addData("runtime", runtime.seconds());
        telemetry.addData("claw position", claw.getPositionDegrees());

//        telemetry.addData("HeadingPID Proportion",bartholomewChassis.HeadingPID.P);
        telemetry.addData("HeadingPID error",chassis.HeadingPID.error);
//        telemetry.addData("TranslationalX Proportion",bartholomewChassis.TranslationalPID_X.P);
        telemetry.addData("TranslationalX error",chassis.TranslationalPID_X.error);
//        telemetry.addData("TranslationalY Proportion",bartholomewChassis.TranslationalPID_Y.P);
        telemetry.addData("TranslationalY error",chassis.TranslationalPID_Y.error);
        telemetry.addData("Distance to Pose", targetPose.getVector2D().findDistance(chassis.getPoseVector().getVector2D()));
        telemetry.addData("loop times", currentTime - lastTime);
    }

    public boolean withinPoseTolerance(){
        return (Math.abs(targetPose.getVector2D().findDistance(chassis.getPoseVector().getVector2D())) < translationalTolerance) && (Math.abs(chassis.HeadingPID.error) < headingTolerance);
    }
}
