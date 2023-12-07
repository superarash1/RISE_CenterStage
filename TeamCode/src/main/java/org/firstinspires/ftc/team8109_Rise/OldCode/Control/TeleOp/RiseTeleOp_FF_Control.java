package org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Control.PIDF_Controller;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.Arm;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.Cage;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.MecanumDriveTrain_Old;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.Gate;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.OpTake;

public class RiseTeleOp_FF_Control {

    MecanumDriveTrain_Old driveTrain;
//
//    // Define Webcam
//    OpenCvCamera webcam;
//
//    // Create Pipeline
//    static BlockDetection pipeline;


    Arm armLeft;
    Arm armRight;
    Gate gate;
    OpTake intake;
    Cage cage;

    Gamepad gamepad1;
    Telemetry telemetry;

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

    double antiGravity;
    double kGravity; //0.2

    double targetPos;
    double maxError;

    double armPower;

    boolean switchMode1 = true;
    boolean switchMode2 = false;

    boolean armToggle1 = true;
    boolean armToggle2 = false;
    boolean armToggle3 = true;
    boolean armToggle4 = false;
    boolean armToggle5 = true;
    boolean armToggle6 = false;
    boolean armToggle7 = true;
    boolean armToggle8 = false;

    boolean toggle1 = true;
    boolean toggle2 = false;

    boolean gateToggle1 = true;
    boolean gateToggle2 = false;

    boolean lastToggleA = true;
    boolean lastToggleX = true;
    boolean lastToggleB = true;
    boolean lastToggleDown = true;
    boolean lastToggleUp = true;
    boolean lastToggleY = true;

    double gatePosition = 0;

    public enum cageState {
        DROP,
        FLAT,
        MANUAL
    }

    public enum armState {
        HOME,
        TOP_LEVEL,
        MIDDLE_LEVEL,
        BOTTOM_LEVEL,
        MANUAL
    }

    public enum driveState {
        TURN,
        CORRECTION,
        STOP
    }

    public driveState DriveState;

    public PIDF_Controller PIDF_Drive;
    public PIDF_Controller PIDF_Turn;

    public armState ArmState;
    public cageState CageState;

    PIDF_Controller ArmPID;

    public double boxTarget;

    public RiseTeleOp_FF_Control(String flName, String frName, String brName, String blName, String Arm1, String Arm2, String CageName, String GateName, String IntakeName, HardwareMap hardwareMap , Telemetry telemetry, Gamepad gamepad1){
        driveTrain = new MecanumDriveTrain_Old(flName, frName, brName, blName, hardwareMap);
        armLeft = new Arm(Arm1, -37, 10.0/42.0, hardwareMap);
        armRight = new Arm(Arm2, -37, 10.0/42.0, hardwareMap);
        gate = new Gate(GateName, hardwareMap, 0.8, 0.35, Gate.gateState.OPEN);
        intake = new OpTake(IntakeName, hardwareMap);
        cage = new Cage(CageName, hardwareMap);
        PIDF_Drive = new PIDF_Controller(0.8, 0.00001); //0.00015
        PIDF_Turn = new PIDF_Controller(0.6);
//
//        // Set up webcam
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        // Set up pipeline
//        pipeline = new BlockDetection(telemetry);
//        webcam.setPipeline(pipeline);
//
//        // Start camera streaming
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });

        ArmPID = new PIDF_Controller(1.4, 0.00001, 0, 0.001); //1, 0, 0, 0.001   1, 0, 0, 0.0005

        ArmPID.tolerance = 0.25;
        armLeft.armMotor.setDirectionReverse();

        driveTrain.setBreakMode();
        driveTrain.reset();

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;

        PIDF_Drive.tolerance = 0.05;
        PIDF_Turn.tolerance = 0.05;

        ArmState = armState.HOME;
        CageState = cageState.FLAT;
    }

    public void teleOpDrive(){

        drive = Math.pow(gamepad1.left_stick_y, 3); //Between -1 and 1
        turn = Math.pow(gamepad1.right_stick_x, 3);
        strafe = Math.pow(gamepad1.left_stick_x, 3);

        if ((gamepad1.a != lastToggleA) && gamepad1.a && toggle1){
            switchMode1 = false;
            switchMode2 = true;
//
//            gate.GateState = Gate.gateState.CLOSED;
//            DriveState = driveState.TURN;

            toggle1 = false;
            toggle2 = true;
        } else if ((gamepad1.a != lastToggleA) && gamepad1.a && toggle2){
            switchMode2 = false;
            switchMode1 = true;

            toggle2 = false;
            toggle1 = true;
        }

        lastToggleA = gamepad1.a;

        // Mecanum Drive Calculations
        if (switchMode1){
            fLeft = 0.875 * drive - 1 * strafe - 0.8 * turn;
            fRight = 0.875 * drive + 1 * strafe + 0.8 * turn;
            bRight = 0.875 * drive - 1 * strafe + 0.8 * turn;
            bLeft = 0.875 * drive + 1 * strafe - 0.8 * turn;
        } else if (switchMode2){
            fLeft = -0.875 * drive + 1 * strafe - 0.8 * turn;
            fRight = -0.875 * drive - 1 * strafe + 0.8 * turn;
            bRight = -0.875 * drive + 1 * strafe + 0.8 * turn;
            bLeft = -0.875 * drive - 1 * strafe - 0.8 * turn;
        }

//        if (switchMode1){
//            drive = Math.pow(gamepad1.left_stick_y, 3); //Between -1 and 1
//            turn = Math.pow(gamepad1.right_stick_x, 3);
//            strafe = Math.pow(gamepad1.left_stick_x, 3);
//
//        } else if (switchMode2){
//            switch (DriveState){
//                case TURN:
//                    turn = 0.4;
//
//                    if (pipeline.yellowContourCount != 0) DriveState = driveState.CORRECTION;
//
//                    break;
//                case CORRECTION:
//                    if (!pipeline.YellowRect.empty()){
//                        boxPositionX = pipeline.YellowRect.x + (pipeline.YellowRect.width/2);
//                        boxPositionY = pipeline.YellowRect.y + (pipeline.YellowRect.height/2);
//
//                        drive = PIDF_Drive.PIDF_Power(boxPositionY, 220, 220); //0.00002
//                        turn = -PIDF_Turn.PIDF_Power(boxPositionX, 160, 320);
//                    }
//                    break;
//            }
//        }
//
//        fLeft = -0.875 * drive + 1 * strafe - 0.8 * turn;
//        fRight = -0.875 * drive - 1 * strafe + 0.8 * turn;
//        bRight = -0.875 * drive + 1 * strafe + 0.8 * turn;
//        bLeft = -0.875 * drive - 1 * strafe - 0.8 * turn;

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

    public void Arm(){
        kGravity = 0.14;
        antiGravity = kGravity*(Math.cos(Math.toRadians(armLeft.armAngle())));
//
        armPower = ArmPID.PIDF_Power(armLeft.armAngle(), targetPos) +  antiGravity;

        if (armLeft.armAngle() < 50){
            CageState = cageState.FLAT;
        }

        if(armLeft.armAngle() > 50){
            CageState = cageState.DROP;
        }

        switch (ArmState){
            case MANUAL:
                if (gamepad1.dpad_up) {
                    armPower = antiGravity + 0.3;
                } else if (gamepad1.dpad_down){
                    armPower = antiGravity - 0.3;
                } else {
                    armPower = antiGravity;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b && armToggle8){
                    armToggle7 = false;
                    armToggle8 = true;

                    ArmState = armState.HOME;
                    CageState = cageState.FLAT;
                }

            case HOME:
                targetPos = -33.5;

                if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle1){
                    armToggle1 = false;
                    armToggle2 = true;

                    ArmState = armState.TOP_LEVEL;
                }

                if ((gamepad1.b != lastToggleB) && gamepad1.b && armToggle7){
                    armToggle7 = false;
                    armToggle8 = true;

                    ArmState = armState.MANUAL;
//                    CageState = cageState.MANUAL;
                }

                lastToggleB = gamepad1.b;
                lastToggleX = gamepad1.x;

                break;
            case TOP_LEVEL:
                targetPos = 135;

                if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle2){
                    armToggle2 = false;
                    armToggle1 = true;

                    ArmState = armState.HOME;
                    CageState = cageState.FLAT;
                }

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down && armToggle3){
                    armToggle3 = false;
                    armToggle4 = true;

                    ArmState = armState.MIDDLE_LEVEL;
                    CageState = cageState.DROP;
                }

                lastToggleX = gamepad1.x;
                lastToggleDown = gamepad1.dpad_down;

                break;
            case MIDDLE_LEVEL:
                targetPos = 175;

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down && armToggle5){
                    armToggle5 = false;
                    armToggle6 = true;

                    ArmState = armState.BOTTOM_LEVEL;
                    CageState = cageState.DROP;
                }

                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up && armToggle4){
                    armToggle4 = false;
                    armToggle3 = true;

                    ArmState = armState.TOP_LEVEL;
                    CageState = cageState.DROP;
                }

                lastToggleDown = gamepad1.dpad_down;
                lastToggleUp = gamepad1.dpad_up;

                break;
            case BOTTOM_LEVEL:
                targetPos = 205;

                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up && armToggle6){
                    armToggle6 = false;
                    armToggle5 = true;

                    ArmState = armState.MIDDLE_LEVEL;
                    CageState = cageState.DROP;
                }
                lastToggleUp = gamepad1.dpad_up;

                break;
        }

        armLeft.armMotor.setPower(armPower);
        armRight.armMotor.setPower(-armPower);
    }

    public void cageRotation(){
        cage.cageSpin.setPosition(boxTarget);

        switch (CageState){
            case FLAT:
                boxTarget = 0.685;
                break;
            case DROP:
                boxTarget = 0;
                break;
            case MANUAL:
                if (gamepad1.right_bumper) {
                    boxTarget -= 0.01;
                } else if (gamepad1.left_bumper) {
                    boxTarget += 0.01;
                }
                break;
        }
    }

    public void teleOpGate(){

        switch (gate.GateState){
            case OPEN:

                if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle1){
                    gateToggle1 = false;
                    gateToggle2 = true;

                    gate.GateState = Gate.gateState.CLOSED;
                }

                lastToggleY = gamepad1.y;
                break;
            case CLOSED:
                if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle2){
                    gateToggle2 = false;
                    gateToggle1 = true;

                    gate.GateState = Gate.gateState.OPEN;
                }

                lastToggleY = gamepad1.y;
                break;
        }
        gate.toggleGate();
    }

    public void teleOpIntake(){
        intake.intakeMotor.setPower(Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3));
    }

    public void Telemetry(){
        telemetry.addData("Gate State", gate.GateState);

        telemetry.addData("Arm Position", armLeft.armAngle());
        telemetry.addData("Arm Target", targetPos);
        telemetry.addData("Arm PID Output", ArmPID.PIDF_Power(armLeft.armAngle(), targetPos));
        telemetry.addData("ArmPID Proportion", ArmPID.P);
        telemetry.addData("ArmPID Derivative", ArmPID.D);
        telemetry.addData("ArmPID Integral", ArmPID.I);

        telemetry.addData("Intake Position", intake.IntakeAngle());
        telemetry.addData("Cage Position", cage.cageSpin.getPosition());

        telemetry.update();
    }
}
