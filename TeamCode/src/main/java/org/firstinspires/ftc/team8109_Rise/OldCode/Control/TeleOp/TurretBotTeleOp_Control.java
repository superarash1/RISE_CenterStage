package org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp;

import android.graphics.Color;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team8109_Rise.Control.PID_Controller;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.MecanumDriveTrain_Old;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.Turret;
import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Sensors.IMU;
import org.firstinspires.ftc.team8109_Rise.Sensors.Camera.OpenCV.VisionPipelines.BlockDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class TurretBotTeleOp_Control {

    OpenCvCamera webcam;

    ElapsedTime runtime  = new ElapsedTime();

    Telemetry telemetry;

    Turret turret;
    MecanumDriveTrain_Old driveTrain;
    IMU inertialMeasurementUnit;

    Gamepad gamepad1;

    double colorValue = 100;

    // Create Pipeline
    public static BlockDetection pipeline;

    double boxPositionX;
    boolean lastToggleX = true;

    double power;

    double drive;
    double turn;
    double strafe;
    double fLeft;
    double fRight;
    double bLeft;
    double bRight;
    double max;

    double TimeMultiplier = 1;

    int red = 1000000;
    int yellow = 2000000;
    int green = 3000000;
    int blue = 4000000;

    int colorCounter = 1000000;
    double deltaTime;

    double kF = 0.7;

    public enum turretState {
        MANUAL,
        TRACKING,
        FIXED
    }

    public turretState TurretState;

    public PID_Controller TurretPID;
    public PID_Controller TurretStationPID;

    public TurretBotTeleOp_Control(String flName, String frName, String brName, String blName, String turretName, HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry){

        PhotonCore.enable();

        TurretPID = new PID_Controller(1, 0.00001);
        TurretStationPID = new PID_Controller(0.8, 0);

        inertialMeasurementUnit = new IMU(hardwareMap);

        driveTrain = new MecanumDriveTrain_Old(flName, frName, brName, blName, hardwareMap);

        driveTrain.setBreakMode();
        driveTrain.reset();

        turret = new Turret(turretName, hardwareMap);

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

        TurretPID.tolerance = 0.5;

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        TurretState = turretState.MANUAL;
    }

    public void ControlHubColor(){

        if (colorCounter > red && colorCounter < yellow){
            colorValue = Color.RED;
        } else if (colorCounter > yellow && colorCounter < green){
            colorValue = Color.YELLOW;
        } else if (colorCounter > green && colorCounter < blue){
            colorValue = Color.GREEN;
        } else if (colorCounter > blue && colorCounter < 5000000){
            colorValue = Color.BLUE;
        } else if (colorCounter > 5000000){
            colorCounter = 1000000;
        }

        PhotonCore.CONTROL_HUB.setConstant((int) colorValue);
//        colorValue = -(Math.pow(Math.abs(colorValue), 1.1));
//        colorValue = colorValue - 10000;
        colorCounter+=50000;
    }

    public void teleOpDrive(){
        drive = Math.pow(gamepad1.left_stick_y, 3); //Between -1 and 1
        turn = Math.pow(gamepad1.right_stick_x, 3);
        strafe = Math.pow(gamepad1.left_stick_x, 3);

        // Mecanum Drive Calculations
        fLeft = -0.875 * drive + 1 * strafe + 0.8 * turn;
        fRight = -0.875 * drive - 1 * strafe - 0.8 * turn;
        bRight = -0.875 * drive + 1 * strafe - 0.8 * turn;
        bLeft = -0.875 * drive - 1 * strafe + 0.8 * turn;

        // This ensures that the power values the motors are set to are in the range (-1, 1)
        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }



        driveTrain.setPower(fLeft, fRight, bRight, bLeft);
    }

    public void turretSpin(){

        switch (TurretState){
            case MANUAL:
                power = Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3);

                if (gamepad1.x != lastToggleX && gamepad1.x){
                    TurretState = turretState.TRACKING;
                }

                lastToggleX = gamepad1.x;

                break;
            case TRACKING:
                if (!pipeline.YellowRect.empty()){
                    boxPositionX = pipeline.YellowRect.x + (pipeline.YellowRect.width/2);

                    power = -TurretPID.PID_Power(boxPositionX, 160);
                }

                if (gamepad1.x != lastToggleX && gamepad1.x){
                    TurretState = turretState.MANUAL;
                }

                lastToggleX = gamepad1.x;

                break;

            case FIXED:
                power = TurretPID.PID_Power(turret.TurretAngle() + inertialMeasurementUnit.Angle_FieldCentric(), 0) - turn * kF;
        }

        turret.turretMotor.setPower(power);
    }

    public void Telemetry(){

        deltaTime = runtime.seconds();
        telemetry.addData("delta time", deltaTime);
        runtime.reset();

        telemetry.addData("Color", colorCounter);

        telemetry.addData("Power", power);
        telemetry.addData("Proportion", TurretPID.P);
        telemetry.addData("Derivative", TurretPID.D);
        telemetry.addData("Turret Position",turret.TurretAngle() + inertialMeasurementUnit.Angle_FieldCentric());
    }

    public void closeCamera(){
        webcam.closeCameraDevice();
    }
}
