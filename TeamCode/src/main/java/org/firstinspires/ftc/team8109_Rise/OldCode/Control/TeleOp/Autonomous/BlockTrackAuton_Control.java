package org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp.Autonomous;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team8109_Rise.Control.PIDF_Controller;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.MecanumDriveTrain_Old;
import org.firstinspires.ftc.team8109_Rise.Sensors.Camera.OpenCV.VisionPipelines.BlockDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class BlockTrackAuton_Control {
    public MecanumDriveTrain_Old chassis;

    // Define Webcam
    OpenCvCamera webcam;

    // Create Pipeline
    static BlockDetection pipeline;

    public Telemetry telemetry;
    public Gamepad gamepad1;

    double drive;
    double turn;
    double strafe;
    double fLeft;
    double fRight;
    double bLeft;
    double bRight;
    double max;

    double boxPositionX;
    double boxPositionY;

    public double power;

    public enum driveState {
        TURN,
        CORRECTION,
        STOP
    }

    public driveState DriveState;

    // TODO: Check if I can move inside constructor
    public PIDF_Controller PIDF_Drive;
    public PIDF_Controller PIDF_Turn;

    public BlockTrackAuton_Control(String flName, String frName, String brName, String blName, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1){

        PIDF_Drive = new PIDF_Controller(0.8, 0.00001); //0.00015
        PIDF_Turn = new PIDF_Controller(0.6);

        // Set up webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up pipeline
        pipeline = new BlockDetection(telemetry);
        webcam.setPipeline(pipeline);

        // Start camera streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        this.chassis = new MecanumDriveTrain_Old(flName, frName, brName, blName, hardwareMap);

        this.chassis.reset();

        PIDF_Drive.tolerance = 0.05;
        PIDF_Turn.tolerance = 0.05;

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;

        DriveState = driveState.TURN;
    }

    //x: 320
    //y: 240 max (target like 200 or smth?? or maybe until off the screen) (maybe after off screen it looks for a second block)

    public void Drive(){

        switch (DriveState){
            case TURN:
                turn = 0.2;

                if (pipeline.yellowContourCount != 0) DriveState = driveState.CORRECTION;

                break;
            case CORRECTION:
                if (!pipeline.YellowRect.empty()){
                    boxPositionX = pipeline.YellowRect.x + (pipeline.YellowRect.width/2);
                    boxPositionY = pipeline.YellowRect.y + (pipeline.YellowRect.height/2);

                    drive = PIDF_Drive.PIDF_Power(boxPositionY, 220); //0.00002
                    turn = -PIDF_Turn.PIDF_Power(boxPositionX, 160);
                }
                break;
        }

        fLeft = drive - strafe + turn;
        fRight = drive + strafe - turn;
        bRight = drive - strafe - turn;
        bLeft = drive + strafe + turn;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        chassis.setPower(fLeft, fRight, bRight, bLeft);
    }

    public void closeCamera(){
        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                webcam.closeCameraDevice();
                webcam.stopStreaming();
            }
        });
    }

    public void Telemetry(){
        telemetry.addData("Auton State", DriveState);
    }
}
