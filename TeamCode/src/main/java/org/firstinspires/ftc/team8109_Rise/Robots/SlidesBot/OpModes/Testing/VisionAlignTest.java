package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team8109_Rise.Math.Vectors.Vector2D;
import org.firstinspires.ftc.team8109_Rise.Math.Vectors.Vector3D;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.Chassis;
import org.firstinspires.ftc.team8109_Rise.Sensors.Camera.OpenCV.VisionPipelines.PowerPlayPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Autonomous
public class VisionAlignTest extends LinearOpMode {
    OpenCvCamera camera; //TODO: Improve tracking
    static PowerPlayPipeline pipeline;
    ElapsedTime runtime = new ElapsedTime();

    Chassis chassis;

    Vector3D targetPose = new Vector3D(0, 0, 0);
    double translationalTolerance = 1;
    double headingTolerance = 0.04;

    enum AutonState{
        IDLE,
        THREE_WHEEL_ODO_PID,
        VISION_PID
    }

    AutonState autonState = AutonState.THREE_WHEEL_ODO_PID;
    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new Chassis(gamepad1, telemetry, hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new PowerPlayPipeline(telemetry);

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

        chassis.TranslationalPID_X.setPIDCoefficients(0.35, 0.035, 0, 0.002);//0.03 0.001
        chassis.TranslationalPID_Y.setPIDCoefficients(0.35, 0.035, 0, 0.002);
        chassis.HeadingPID.setPIDCoefficients(2, 0.02, 0, 0.001); // 0.03 0.1 0.001
        targetPose.set(51.3, -5.2, -0.6362);

        while (opModeInInit()){
            chassis.update();
            chassis.updatePoseEstimate();

            pipeline.Telemetry();
            telemetry.update();
        }

        while (opModeIsActive()){
            chassis.update();
            chassis.updatePoseEstimate();

            switch (autonState){
                case THREE_WHEEL_ODO_PID:
                    chassis.goToPosePID(targetPose);

                    if (withinPoseTolerance()){
                        autonState = AutonState.VISION_PID;
                    }
                    break;
                case VISION_PID:
                    chassis.visionAlign(new Vector2D(100, 160), pipeline);

                    if (withinCorrectionTolerance()){
                        autonState = AutonState.IDLE;
                    }
                    break;
                case IDLE:
                    chassis.setPower(0);
                    break;
            }
        }
    }

    public boolean withinPoseTolerance(){
        return (Math.abs(targetPose.getVector2D().findDistance(chassis.getPoseVector().getVector2D())) < translationalTolerance) && (Math.abs(chassis.HeadingPID.error) < headingTolerance);
    }

    public boolean withinCorrectionTolerance(){
        return (Math.abs(chassis.visionX_PID.error) < chassis.visionX_PID.tolerance) && (Math.abs(chassis.visionHeading_PID.error) < chassis.visionHeading_PID.tolerance);
    }

}