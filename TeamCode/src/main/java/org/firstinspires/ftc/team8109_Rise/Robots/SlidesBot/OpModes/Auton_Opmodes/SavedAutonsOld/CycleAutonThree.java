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
import org.firstinspires.ftc.team8109_Rise.Sensors.Camera.OpenCV.VisionPipelines.ColorPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class CycleAutonThree extends LinearOpMode {
    OpenCvCamera camera; //TODO: Improve tracking
    ColorPipeline pipeline;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime globalTime = new ElapsedTime();

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
        BOOST_UP,
        TO_HIGH_JUNCTION,
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

        autonState = AutonState.PUSH_CONE;
        cycleState = CycleState.TO_CONE_STACK;
        parkingStep = ParkingStep.STEP_ONE;

        chassis = new Chassis(gamepad1, telemetry, hardwareMap);
        slides = new ViperSlides(gamepad1, telemetry, hardwareMap);
        arm = new ServoIntakeArm(gamepad1,telemetry, hardwareMap);
        wrist = new Wrist(gamepad1, hardwareMap);
        claw = new Claw(gamepad1, telemetry, hardwareMap);
        odoRetract = new OdoRetract(gamepad1, hardwareMap);

        claw.clawState = ServoClaw.ClawState.CLOSED;
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

        while (opModeInInit()){
            claw.setPosition();
            odoRetract.podState = OdoRetract.PodState.GROUND;

            if (pipeline.findColor() == ColorPipeline.Colors.BLUE){
                parkingZone = ParkingZone.LEFT;
            }

            if (pipeline.findColor() == ColorPipeline.Colors.RED){
                parkingZone = ParkingZone.RIGHT;
            }

            if (pipeline.findColor() == ColorPipeline.Colors.GREEN){
                parkingZone = ParkingZone.MIDDLE;
            } else if (pipeline.findColor() == ColorPipeline.Colors.UNKNOWN){
                parkingZone = ParkingZone.MIDDLE;
            }

            telemetry.addData("color", pipeline.findColor());
            telemetry.addData("color", pipeline.getHue());

            telemetry.update();
        }

        globalTime.reset();

        while (opModeIsActive()){
            chassis.update();
            chassis.updatePoseEstimate();

            autonLeftRed();

            if (runtime.seconds() > 15) {
                arm.servoPosition = ServoIntakeArm.ServoPosition.INTAKE_POSITION;
                wrist.wristPosition = Wrist.WristPosition.INTAKE_POSITION;
                slides.slidesState = ViperSlides.SlidesState.GROUND;
                chassis.goToPosePID(chassis.getPoseVector());
            }

            chassis.goToPosePID(targetPose);

            slides.setSlidePower();
            claw.setPosition();
            arm.setArmPosition();
            wrist.setPosition();
            odoRetract.setPodPosition();

            Telemetry();
            telemetry.update();
        }
    }

    //TODO: odo retract can go with chassis control

    public void autonLeftRed(){
        switch (autonState){
            case PUSH_CONE:
                targetPose.set(60, 0, 0);
//
                claw.clawState = ServoClaw.ClawState.CLOSED;

                chassis.goToPosePID(targetPose);

                if (targetPose.findDistance(chassis.getPoseVector()) < tolerance){
                    autonState = AutonState.RETURN_TO_POLE;
                    runtime.reset();
                }
                break;
            case RETURN_TO_POLE:
                targetPose.set(46, 0, -0.773);
                chassis.goToPosePID(targetPose);

                if (targetPose.findDistance(chassis.getPoseVector()) < tolerance){
                    autonState = AutonState.GO_TO_SCORE_PRELOAD;
                    runtime.reset();
                }
                break;
            case GO_TO_SCORE_PRELOAD:
                slides.slidesState = ViperSlides.SlidesState.HIGH_JUNCTION;

                arm.servoPosition = ServoIntakeArm.ServoPosition.OUTTAKE_POSITION;
                wrist.wristPosition = Wrist.WristPosition.OUTTAKE_POSITION;
                targetPose.set(54.5, -3.5, -0.959);

                chassis.goToPosePID(targetPose);

                // could also vector sum all errors
                if (targetPose.findDistance(chassis.getPoseVector()) < tolerance){
                    autonState = AutonState.SCORE_PRELOAD;
                    runtime.reset();
                }
                break;
            case SCORE_PRELOAD:
                targetPose.set(54.5, -3.5, -0.959);

                if (runtime.seconds() > 0.5){
                    claw.clawState = ServoClaw.ClawState.OPEN;
                }

                // if ((claw.getPositionDegrees() > 175) && runtime.seconds() > 2){
                if ((claw.getPositionDegrees() > 140)){
                    autonState = AutonState.CYCLE;
                    runtime.reset();
                }
                break;
            case CYCLE:
                switch (cycleState){
                    case TO_CONE_STACK:
                        targetPose.set(47.75, 26.8, -Math.toRadians(90));

                        switch (cycleCounter){
                            case 1:
                                slides.slidesState = ViperSlides.SlidesState.CONESTACK_TOP;
                                break;
                            case 2:
                                slides.slidesState = ViperSlides.SlidesState.CONESTACK_TOP_MIDDLE;
                                break;
                            case 3:
                                slides.slidesState = ViperSlides.SlidesState.CONESTACK_MIDDLE;
                                break;
                            case 4:
                                slides.slidesState = ViperSlides.SlidesState.CONESTACK_BOTTOM_MIDDLE;
                                break;
                            case 5:
                                slides.slidesState = ViperSlides.SlidesState.GROUND;
                                break;
                        }

                        arm.servoPosition = ServoIntakeArm.ServoPosition.INTAKE_POSITION;
                        wrist.wristPosition = Wrist.WristPosition.INTAKE_POSITION;
                        claw.clawState = ServoClaw.ClawState.OPEN;

                        if (targetPose.findDistance(chassis.getPoseVector()) < tolerance){
                            cycleState = CycleState.PICK_UP_CONE;
                            runtime.reset();
                        }
                        break;

                    case PICK_UP_CONE:

                        //TODO: have arm and slide also move a certain amount before going to next state in order to
                        targetPose.set(47.75, 26.8, -Math.toRadians(90));
                        claw.clawState = ServoClaw.ClawState.CLOSED;

                        // TODO: Note that the claw won't actually reach this position closing
                        //runtime.seconds timer maybe needed
                        if ((claw.getPositionDegrees() < 110) && runtime.seconds() > 0.5) {
                            cycleState = CycleState.BOOST_UP;
                            runtime.reset();
                        }
                        break;

                    case BOOST_UP:
                        slides.slidesState = ViperSlides.SlidesState.HIGH_JUNCTION;

                        if (runtime.seconds() > 0.75) {
                            cycleState = CycleState.TO_HIGH_JUNCTION;
                            runtime.reset();
                        }
                        break;
                    case TO_HIGH_JUNCTION:
                        targetPose.set(48.18, 13, -Math.toRadians(90));

                        targetPose.set(53, -2, -0.959);


//                        targetPose.set(53, -2, -0.959);
                        slides.slidesState = ViperSlides.SlidesState.HIGH_JUNCTION;

                        arm.servoPosition = ServoIntakeArm.ServoPosition.OUTTAKE_POSITION;
                        wrist.wristPosition = Wrist.WristPosition.OUTTAKE_POSITION;

                        if (targetPose.findDistance(chassis.getPoseVector()) < tolerance){
                            cycleState = CycleState.SCORE_CONE;
                            runtime.reset();
                        }
                        break;
                    case SCORE_CONE:
                        targetPose.set(54.5, -3.5, -0.959);

                        if (runtime.seconds() > 0.75){
                            claw.clawState = ServoClaw.ClawState.OPEN;
                        }

                        if ((claw.getPositionDegrees() > 140)/* && runtime.seconds() > 0.5*/){
                            cycleCounter++;

                            cycleState = CycleState.TO_CONE_STACK;
                            runtime.reset();
                        }
                        break;
                }
                if ((globalTime.seconds() > 28) || cycleCounter > 3){
                    autonState = AutonState.PARK;
                }
                break;
            case PARK:
                slides.slidesState = ViperSlides.SlidesState.GROUND;
                arm.servoPosition = ServoIntakeArm.ServoPosition.INTAKE_POSITION;
                wrist.wristPosition = Wrist.WristPosition.INTAKE_POSITION;
                claw.clawState = ServoClaw.ClawState.OPEN;

                switch (parkingStep){
                    case STEP_ONE:
                        targetPose.set(45, 0, 0);

                        if (targetPose.findDistance(chassis.getPoseVector()) < tolerance){
                            parkingStep = ParkingStep.STEP_TWO;
                            runtime.reset();
                        }
                        break;
                    case STEP_TWO:
                        targetPose.set(26, 0, 0);
                        if (targetPose.findDistance(chassis.getPoseVector()) < tolerance){
                            parkingStep = ParkingStep.STEP_THREE;
                            runtime.reset();
                        }
                        break;
                    case STEP_THREE:
                        switch (parkingZone){
                            case LEFT:
                                targetPose.set(26, 23, 0);
                                break;
                            case MIDDLE:
                                targetPose.set(26, 0, 0);

                                break;
                            case RIGHT:
                                targetPose.set(26, -23, 0);
                                break;
                        }
                }


//                switch (parkingZone){
//                    case LEFT:
//                        targetPose.set(45, 22, -Math.toRadians(90));
//                        break;
//                    case MIDDLE:
//                        targetPose.set(45, -3, -Math.toRadians(90));
//                        break;
//                    case RIGHT:
//                        targetPose.set(45, -27, -Math.toRadians(90));
//                        break;
//                }
        }
    }

    public void Telemetry(){
        //TODO: show the tolerance stuff and PID values and states
        telemetry.addData("Auton State", autonState);
        telemetry.addData("Cycle State", cycleState);
        telemetry.addData("cycle count", cycleCounter);

        telemetry.addData("runtime", runtime.seconds());
        telemetry.addData("claw position", claw.getPositionDegrees());

//        telemetry.addData("HeadingPID Proportion",chassis.HeadingPID.P);
        telemetry.addData("HeadingPID error",chassis.HeadingPID.error);
//        telemetry.addData("TranslationalX Proportion",chassis.TranslationalPID_X.P);
        telemetry.addData("TranslationalX error",chassis.TranslationalPID_X.error);
//        telemetry.addData("TranslationalY Proportion",chassis.TranslationalPID_Y.P);
        telemetry.addData("TranslationalY error",chassis.TranslationalPID_Y.error);
        telemetry.addData("Distance to Pose", targetPose.findDistance(chassis.getPoseVector()));
    }
}
