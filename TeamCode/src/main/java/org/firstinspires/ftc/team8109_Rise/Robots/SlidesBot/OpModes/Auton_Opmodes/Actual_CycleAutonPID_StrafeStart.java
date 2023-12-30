package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.Auton_Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous
public class Actual_CycleAutonPID_StrafeStart extends LinearOpMode {

    /*
    TODO:
    - Start Fast, go to tile 4 fast
    - Medium speed return to pole
    - Medium go to score preload
    - LineUp is medium high
    - Decelerate to cone stack (p controller)
    - Medium Move out
    - medium to high junction
    - heading error and translational error separation
     */


    //TODO: Try Photon


    //TODO: things to tune: tolerances, heading position, translational position
    OpenCvCamera camera; //TODO: Improve tracking
    static ColorPipeline pipeline;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime globalTime = new ElapsedTime();
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//
//    static final double FEET_PER_METER = 3.28084;
//
//
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//
//    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
//
//    AprilTagDetection tagOfInterest = null;
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

    Chassis chassis;
    ViperSlides slides;
    ServoIntakeArm arm;
    Wrist wrist;
    Claw claw;
    OdoRetract odoRetract;

    boolean stop = false;
    Vector3D targetPose = new Vector3D(0, 0, 0);
    double translationalTolerance = 0.9;
    double headingTolerance = 0.04;

    @Override
    public void runOpMode() throws InterruptedException {

        autonState = AutonState.PUSH_CONE;
        cycleState = CycleState.TO_CONE_STACK;
        parkingStep = ParkingStep.STEP_THREE;

        chassis = new Chassis(gamepad1, telemetry, hardwareMap);
        slides = new ViperSlides(gamepad1, telemetry, hardwareMap);
        arm = new ServoIntakeArm(gamepad1,telemetry, hardwareMap);
        wrist = new Wrist(gamepad1, hardwareMap);
        claw = new Claw(gamepad1, telemetry, hardwareMap);
        odoRetract = new OdoRetract(gamepad1, hardwareMap);

        arm.servoPosition = ServoIntakeArm.ServoPosition.INTAKE_POSITION;
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

        odoRetract.podState = OdoRetract.PodState.GROUND;
        parkingZone = ParkingZone.RIGHT;

        chassis.setPoseEstimate(new Pose2d(0,0,0));

        while (opModeInInit()){
            odoRetract.setPodPosition();
            slides.setSlidePower();
            claw.setPosition();
            arm.setArmPosition();
            wrist.setPosition();

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
//            if (pipeline.findColor() == ColorPipeline.Colors.UNKNOWN){
//                parkingZone = ParkingZone.MIDDLE;
//            }

            parkingZone = ParkingZone.LEFT;

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
            arm.setArmPosition();
            wrist.setPosition();

            currentTime = loopTimes.milliseconds();
            Telemetry();
            telemetry.update();
            lastTime = currentTime;
        }
    }

    //TODO: odo retract can go with chassis control


    //TODO: move all positions a bit back
    public void autonLeftRed(){
        switch (autonState){
            case PUSH_CONE:
                chassis.TranslationalPID_X.setPIDCoefficients(0.15, 0.01, 0, 0.001);
                chassis.TranslationalPID_Y.setPIDCoefficients(0.15, 0.01, 0, 0.001);
                chassis.HeadingPID.setPIDCoefficients(1.5, 0.02, 0, 0); // 0.03 0.1 0.001
                targetPose.set(-3, 48, Math.toRadians(2));

                if (runtime.seconds() > 0.3){
                    arm.servoPosition = ServoIntakeArm.ServoPosition.OUTTAKE_POSITION;
                    wrist.wristPosition = Wrist.WristPosition.OUTTAKE_POSITION;
                }

                chassis.goToPosePID(targetPose);

                claw.clawState = ServoClaw.ClawState.CLOSED;


                if (withinPoseTolerance()){
                    autonState = AutonState.GO_TO_SCORE_PRELOAD;

//                    chassis.TranslationalPID_X.setPIDCoefficents(0.1, 0.01, 0, 0.002);//0.03 0.001
//                    chassis.TranslationalPID_Y.setPIDCoefficents(0.1, 0.01, 0, 0.002);
//                    chassis.HeadingPID.setPIDCoefficents(2, 0.02, 0, 0); // 0.03 0.1 0.001

                    chassis.TranslationalPID_X.setPIDCoefficients(0.35, 0.0375, 0, 0.0025);//0.03 0.001
                    chassis.TranslationalPID_Y.setPIDCoefficients(0.4, 0.0375, 0, 0.01);
                    chassis.HeadingPID.setPIDCoefficients(2, 0.02, 0, 0.001);
                    runtime.reset();

                    // TODO: Y negative left

//                    if (runtime.seconds() > 0.5){
//                        autonState = AutonState.RETURN_TO_POLE;
//                        runtime.reset();
//                    }
                }
                break;
            case RETURN_TO_POLE:
//                slides.slidesState = ViperSlides.SlidesState.HIGH_JUNCTION;
//
//                arm.servoPosition = ServoIntakeArm.ServoPosition.OUTTAKE_POSITION;
//                wrist.wristPosition = Wrist.WristPosition.OUTTAKE_POSITION;
//                slides.slidesState = ViperSlides.SlidesState.HIGH_JUNCTION;

                // Cone push to left
//               ///////////////////////////////////////////////////////////
//                targetPose.set(49.528, 0.833, -0.42);
//                chassis.goToPosePID(targetPose);
//
//                if (withinPoseTolerance()){
//                    autonState = AutonState.GO_TO_SCORE_PRELOAD;
//                    chassis.TranslationalPID_X.setPIDCoefficents(0.35, 0.0375, 0, 0.0025);//0.03 0.001
//                    chassis.TranslationalPID_Y.setPIDCoefficents(0.4, 0.0375, 0, 0.01);
//                    chassis.HeadingPID.setPIDCoefficents(2, 0.02, 0, 0.001); // 0.03 0.1 0.001
//                    runtime.reset();
//                }
//                break;
            case GO_TO_SCORE_PRELOAD:
                translationalTolerance = 0.2;
                slides.slidesState = ViperSlides.SlidesState.HIGH_JUNCTION;

                arm.servoPosition = ServoIntakeArm.ServoPosition.OUTTAKE_POSITION;
                wrist.wristPosition = Wrist.WristPosition.OUTTAKE_POSITION;
//                targetPose.set(54.888, -8.772, -0.638);
                targetPose.set(3, 56.2, 0.9531); //2.175, 56.2, 0.9531
                // New pos: 54.755, -5.355, -0.707
                // Old: 54.728, -8.144, -0.702
//
                chassis.goToPosePID(targetPose);

                // could also vector sum all errors
                if (withinPoseTolerance() && (runtime.seconds() > 0.75)){
                    autonState = AutonState.DUNK_CONE;
//                    chassis.trackingObject = Chassis.TrackingObject.JUNCTION;
                    runtime.reset();
                }
                break;                    // && (Math.abs(slides.slidesPID.error) < slides.slidesPID.tolerance)

            case DUNK_CONE:
                chassis.goToPosePID(targetPose);

                if (runtime.seconds() > 0.25){
                    slides.slidesState = ViperSlides.SlidesState.HIGH_DUNK;
//                    arm.servoPosition = ServoIntakeArm.ServoPosition.DUNK_POSITION;
                    //Math.abs(slides.slidesPID.error) < slides.slidesPID.tolerance
                    autonState = AutonState.SCORE_PRELOAD;
                    claw.clawState = ServoClaw.ClawState.OPEN;
                    runtime.reset();
                }
                break;
            case SCORE_PRELOAD:claw.clawState = ServoClaw.ClawState.OPEN;
                //TODO: Tune these numbers
//                chassis.visionAlign(new Vector2D(209, 143), pipeline); //130, 151
//                chassis.visionAlign(new Vector2D(150, 150), pipeline);

//                targetPose.set(54.888, -8.772, -0.638);
                chassis.goToPosePID(targetPose);
//                pipeline.Telemetry();
                if ((claw.getPositionDegrees() > 140) && runtime.seconds() > 0.1){
                    autonState = AutonState.CYCLE;
                    runtime.reset();
                    translationalTolerance = 1;
                }

                // if ((claw.getPositionDegrees() > 175) && runtime.seconds() > 2){

//                slides.slidesPID.tolerance = 0.1;
//                if (withinPoseTolerance() && (Math.abs(slides.slidesPID.error) < slides.slidesPID.tolerance)){
//                    slides.slidesState = ViperSlides.SlidesState.MIDDLE_JUNCTION;
//                    claw.clawState = ServoClaw.ClawState.OPEN;
//
//                    translationalTolerance = 0.9;
//                    runtime.reset();
//                }

                // if ((claw.getPositionDegrees() > 175) && runtime.seconds() > 2){

//                if (runtime.seconds() > 0.25 && withinCorrectionTolerance() && (Math.abs(slides.slidesPID.error) < slides.slidesPID.tolerance)){
//
//                }

                if ((claw.getPositionDegrees() > 140) && (Math.abs(slides.slidesPID.error) < slides.slidesPID.tolerance)) {
                    autonState = AutonState.CYCLE;
                    runtime.reset();
                }

                // 51.505, 12.355, -1.608654

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

                        if (withinPoseTolerance()){
                            cycleState = CycleState.PICK_UP_CONE;
                            runtime.reset();
                            translationalTolerance = 0.9;

//                            chassis.trackingObject = Chassis.TrackingObject.CONESTACK;
                        }
                        break;

                    case PICK_UP_CONE:

                        //TODO: have arm and slide also move a certain amount before going to next state in order to
//                        chassis.visionAlign(new Vector2D(100, 160), pipeline);
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
                        claw.clawState = ServoClaw.ClawState.CLOSED;

                        if (runtime.seconds() > 0.25) {
                            chassis.TranslationalPID_X.setPIDCoefficients(0.13, 0.0375, 0, 0.001);//0.03 0.001
                            chassis.TranslationalPID_Y.setPIDCoefficients(0.13, 0.0375, 0, 0.001);
                            chassis.HeadingPID.setPIDCoefficients(1.5, 0.03, 0, 0.001);

//                            chassis.TranslationalPID_X.setPIDCoefficents(0.15, 0.0375, 0, 0.001);//0.03 0.001
//                            chassis.TranslationalPID_Y.setPIDCoefficents(0.2, 0.0375, 0, 0.001);
//                            chassis.HeadingPID.setPIDCoefficents(2.2, 0.03, 0, 0.001);

                            cycleState = CycleState.TO_HIGH_JUNCTION;
                            slides.slidesState = ViperSlides.SlidesState.HIGH_JUNCTION;
                            arm.servoPosition = ServoIntakeArm.ServoPosition.OUTTAKE_POSITION;
                            wrist.wristPosition = Wrist.WristPosition.OUTTAKE_POSITION;

//                            translationalTolerance = 1;
                            runtime.reset();
                        }
                        break;
                    case MOVE_OUT:
//                        targetPose.set(50.3, 10, -1.609);
//                        targetPose.set(50, -4, -1.606);
//                        targetPose.set(53, -2, -0.959);
//                        chassis.setPower(-0.4);

                        // 51.477, 10.488, 270
                        targetPose.set(50.3, 15, -1.609);
//                        targetPose.set(50, -4, -1.606);
//                        targetPose.set(53, -2, -0.959);
//                        chassis.setPower(-0.4);
                        // 51.477, 10.488, 270

                        chassis.setPower(-0.5);

//                        chassis.goToPosePID(targetPose);
                        if (Math.abs(15 - chassis.getPoseEstimate().getY()) < 2){
                            cycleState = CycleState.TO_HIGH_JUNCTION;
                        }


//                        if (/*withinPoseTolerance()*/ runtime.seconds() > 0.5){
//                            cycleState = CycleState.TO_HIGH_JUNCTION;
//                            chassis.TranslationalPID_X.setPIDCoefficents(3, 0.02, 0, 0);//0.03 0.001
//                            chassis.TranslationalPID_Y.setPIDCoefficents(0.2, 0.035, 0, 0);
//                            chassis.HeadingPID.setPIDCoefficents(0.2, 0.05, 0, 0.001); // 0.03 0.1 0.001
//
//                            translationalTolerance = 1;
//                            runtime.reset();
//                        }

                        break;

                    case TO_HIGH_JUNCTION:
                        targetPose.set(3, 56.2, 0.9531); //2.175, 56.2, 0.9531
                        chassis.goToPosePID(targetPose);

//                        slides.slidesState = ViperSlides.SlidesState.HIGH_JUNCTION;

//                        arm.servoPosition = ServoIntakeArm.ServoPosition.OUTTAKE_POSITION;
//                        wrist.wristPosition = Wrist.WristPosition.OUTTAKE_POSITION;

                        //TODO: Copy dunking logic from earlier
                        if (withinPoseTolerance()){
//                            cycleState = CycleState.SCORE_CONE;
                            cycleState = CycleState.DUNK_CONE;

                            runtime.reset();
                        }
                        break;

                    case DUNK_CONE:
                        chassis.goToPosePID(targetPose);
                        if (runtime.seconds() > 0.1){
                            slides.slidesState = ViperSlides.SlidesState.HIGH_DUNK;
                            arm.servoPosition = ServoIntakeArm.ServoPosition.DUNK_POSITION;
                            //Math.abs(slides.slidesPID.error) < slides.slidesPID.tolerance
                            cycleState = CycleState.SCORE_CONE;
                            claw.clawState = ServoClaw.ClawState.OPEN;
                            runtime.reset();
                        }

                        break;
                    case SCORE_CONE:
                        if ((claw.getPositionDegrees() > 140) && runtime.seconds() > 0.2){
                            cycleState = CycleState.TO_CONE_STACK;
                            cycleCounter++;
                            runtime.reset();
                        }
//
//                        if ((claw.getPositionDegrees() > 140) && (Math.abs(slides.slidesPID.error) < slides.slidesPID.tolerance)) {
//                            cycleState = CycleState.TO_CONE_STACK;
//                            cycleCounter++;
//                            runtime.reset();
//                        }

                        break;
                    case WAIT_THREE:
                        if (runtime.seconds() > 1){
                            cycleCounter++;
                            cycleState = CycleState.TO_CONE_STACK;
                            runtime.reset();
                        }
                        break;
                }

                //TODO: Autopark
                if (/*(globalTime.seconds() > 28) ||*/ cycleCounter > 5){
                    autonState = AutonState.PARK;
                }
                break;
            case PARK:
                slides.slidesState = ViperSlides.SlidesState.GROUND;
                arm.servoPosition = ServoIntakeArm.ServoPosition.INTAKE_POSITION;
                wrist.wristPosition = Wrist.WristPosition.INTAKE_POSITION;
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
        telemetry.addData("Parking Zone", parkingZone);
        telemetry.addData("Slides error", slides.slidesPID.error);

        telemetry.addData("Pose Estimate", chassis.getPoseEstimate());
        telemetry.addData("Getting Chassis Pose", chassis.getPoseVector());

        telemetry.addData("runtime", runtime.seconds());
        telemetry.addData("claw position", claw.getPositionDegrees());

//        telemetry.addData("HeadingPID Proportion",chassis.HeadingPID.P);
        telemetry.addData("HeadingPID error",chassis.HeadingPID.error);
//        telemetry.addData("TranslationalX Proportion",chassis.TranslationalPID_X.P);
        telemetry.addData("TranslationalX error",chassis.TranslationalPID_X.error);
//        telemetry.addData("TranslationalY Proportion",chassis.TranslationalPID_Y.P);
        telemetry.addData("TranslationalY error",chassis.TranslationalPID_Y.error);
        telemetry.addData("Distance to Pose", targetPose.getVector2D().findDistance(chassis.getPoseVector().getVector2D()));
        telemetry.addData("loop times", currentTime - lastTime);
    }

    public boolean withinPoseTolerance(){
        return (Math.abs(targetPose.getVector2D().findDistance(chassis.getPoseVector().getVector2D())) < translationalTolerance) && (Math.abs(chassis.HeadingPID.error) < headingTolerance);
    }

    public boolean withinCorrectionTolerance(){
        return (Math.abs(chassis.visionX_PID.error) < chassis.visionX_PID.tolerance) && (Math.abs(chassis.visionHeading_PID.error) < chassis.visionHeading_PID.tolerance);
    }

//    void tagToTelemetry(AprilTagDetection detection) {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }
}
