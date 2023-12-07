package org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team8109_Rise.Control.PIDF_Controller;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.MecanumDriveTrain_Old;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.Turret;
import org.firstinspires.ftc.team8109_Rise.Sensors.Camera.OpenCV.VisionPipelines.PersonTracker;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class PersonTrackControl {

    public MecanumDriveTrain_Old chassis;
    public Turret turret;

    // Define Webcam
    OpenCvCamera webcam;

    static PersonTracker pipeline;

    public Telemetry telemetry;
    public Gamepad gamepad1;

    public ElapsedTime DriveLostTimer = new ElapsedTime();
    public ElapsedTime TurretLostTimer = new ElapsedTime();

    double drive;
    double turn;
    double strafe;
    double fLeft;
    double fRight;
    double bLeft;
    double bRight;
    double max;

    double personPositionX;
    double personPositionY;

    boolean lastToggleX = true;
    boolean lastToggleY = true;

    boolean driveStateToggle1 = true;
    boolean driveStateToggle2 = false;

    boolean turretToggle1 = true;
    boolean turretToggle2 = false;

    public double turretPower;

    public enum driveState {
        DRIVER_CONTROL,
        DRIVE_TRACKING,
        ALIGN,
        IDLE
    }

    public enum turretState {
        IDLE,
        MANUAL,
        TRACKING,
        RETURN,
        TURN_TRACKING
    }

    public enum turretSearch {
        TURN,
        TRACKING
    }

    public driveState DriveState;
    public turretState TurretState;
    public turretSearch TurretSearch;

    public PIDF_Controller PIDF_Drive;
    public PIDF_Controller PIDF_Turn;
    public PIDF_Controller TurretPID;

    public PersonTrackControl(String flName, String frName, String brName, String blName, String turretName, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1){
        PIDF_Drive = new PIDF_Controller(1, 0); //0.00015
        PIDF_Turn = new PIDF_Controller(0.6);

        //TODO: Re-tune for new faster motor
        TurretPID = new PIDF_Controller(1);

        // Set up webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up pipeline
        pipeline = new PersonTracker(telemetry);

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
        turret = new Turret(turretName, hardwareMap);

        this.chassis.reset();

        PIDF_Drive.tolerance = 0.05;
        PIDF_Turn.tolerance = 0.05;
        TurretPID.tolerance = 0.5;

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;

        DriveState = driveState.DRIVER_CONTROL;
        TurretState = turretState.MANUAL;
        TurretSearch = turretSearch.TURN;
    }

    public void Drive(){
        //TODO: Add encoder to turret to auto send back to 0 position before drive tracking (run to position)
        switch (DriveState){
            case DRIVER_CONTROL:
                drive = Math.pow(gamepad1.left_stick_y, 3); //Between -1 and 1
                turn = Math.pow(gamepad1.right_stick_x, 3);
                strafe = Math.pow(gamepad1.left_stick_x, 3);

                if ((gamepad1.x != lastToggleX) && gamepad1.x && driveStateToggle1){
                    driveStateToggle1 = false;
                    driveStateToggle2 = true;

                    DriveLostTimer.reset();

                    TurretState = turretState.RETURN;
                }

                lastToggleX = gamepad1.x;
                break;

            case DRIVE_TRACKING:
                if (!pipeline.TargetRect.empty()){
                    personPositionX = pipeline.getRectCenterX(pipeline.TargetRect);
                    personPositionY = pipeline.getRectCenterY(pipeline.TargetRect);

                    drive = -PIDF_Drive.PIDF_Power(240 - personPositionY, 220); //0.00002
                    turn = -PIDF_Turn.PIDF_Power(personPositionX, 160);
                    strafe = 0;

                    DriveLostTimer.reset();
                }
//
//                if (DriveLostTimer.seconds() > 2){
//                    TurretState = turretState.TURN_TRACKING;
//                    TurretSearch = turretSearch.TURN;
//                    DriveState = driveState.IDLE;
//                }

                if ((gamepad1.x != lastToggleX) && gamepad1.x && driveStateToggle2){
                    driveStateToggle1 = false;
                    driveStateToggle2 = true;

                    TurretState = turretState.MANUAL;
                    DriveState = driveState.DRIVER_CONTROL;
                }
                lastToggleX = gamepad1.x;
                break;

            case IDLE:
                drive = 0;
                turn = 0;
                strafe = 0;
                break;

            case ALIGN:

                turn = PIDF_Turn.PIDF_Power(turret.TurretAngle(), 0);

                if (Math.abs(PIDF_Turn.error) < PIDF_Turn.tolerance){
                    TurretState = turretState.RETURN;
                    DriveState = driveState.DRIVE_TRACKING;
                }
                break;
        }

        // Mecanum Drive Calculations
        fLeft = -drive + strafe + turn;
        fRight = -drive - strafe - turn;
        bRight = -drive + strafe - turn;
        bLeft = -drive - strafe + turn;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        chassis.setPower(fLeft, fRight, bRight, bLeft);
    }

    public void Turret(){
        switch (TurretState){
            case MANUAL:
                turretPower = Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3);

                if (gamepad1.y != lastToggleY && gamepad1.y && turretToggle1){
                    TurretState = turretState.TRACKING;

                    turretToggle1 = false;
                    turretToggle2 = true;
                }

                lastToggleY = gamepad1.y;
                break;

            case TRACKING:
                if (!pipeline.TargetRect.empty()){
                    personPositionX = pipeline.getRectCenterX(pipeline.TargetRect);

                    turretPower = -TurretPID.PIDF_Power(personPositionX, 160);
                }

                if (gamepad1.y != lastToggleY && gamepad1.y && turretToggle2){
                    TurretState = turretState.MANUAL;

                    turretToggle2 = false;
                    turretToggle1 = true;
                }

                lastToggleY = gamepad1.y;
                break;

            // TODO: add a state to return the initial angle when vision drive turns on and PID there until TURN_TRACKING
            case RETURN:
                turretPower = TurretPID.PIDF_Power(turret.TurretAngle(), 0);

                if (Math.abs(TurretPID.error) < TurretPID.tolerance){
                    DriveState = driveState.DRIVE_TRACKING;
                }
                break;

            case TURN_TRACKING:
                switch (TurretSearch){
                    case TURN:
                        turretPower = -0.2; // TODO: follow bot's previous direction (0.2*Math.signum(PIDF_Turn.error))

                        if (!pipeline.TargetRect.empty()){
                            TurretLostTimer.reset();
                            TurretSearch = turretSearch.TRACKING;
                        }

                        break;
                    case TRACKING: // TODO: keep on PID Tracking mode until bot re-aligns
                        if (!pipeline.TargetRect.empty()){
                            personPositionX = pipeline.getRectCenterX(pipeline.TargetRect);

                            turretPower = -TurretPID.PIDF_Power(personPositionX, 160);
                            TurretLostTimer.reset();
                        }

                        if (Math.abs(TurretPID.error) < TurretPID.tolerance) {
                            DriveState = driveState.ALIGN;
                            //TODO: potentially add another state to have the turret to turn inverse to bot if camera PID fails (feedforward? Relative to turn speed?)
                        }
                        //TODO: return turret to IDLE state after drivetrain finishes align mode

                        break;
                }

            case IDLE:
                turret.turretMotor.setPower(0);
                break;
        }

        turret.turretMotor.setPower(turretPower);
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
        telemetry.addData("Drive State", DriveState);
        telemetry.addData("Drive P", PIDF_Drive.P);
        telemetry.addData("Heading P", PIDF_Turn.P);
        telemetry.addData("Turret P", TurretPID.P);

        telemetry.addData("Drive State", DriveState);
        telemetry.addData("Turret State", TurretState);
        telemetry.addData("Turret Search State", TurretSearch);
    }
}
