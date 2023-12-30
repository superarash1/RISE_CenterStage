package org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Control.PID_Controller;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.MecanumDriveTrain_Old;

public class BotToPoint_Control {
    MecanumDriveTrain_Old driveTrain;

    Gamepad gamepad1;

    Telemetry telemetry;

    double power;

    double drive;
    double turn;
    double strafe;
    double fLeft;
    double fRight;
    double bLeft;
    double bRight;
    double max;

    public enum driveState{
        MANUAL,
        TO_POINT
    }

    driveState DriveState;

    PID_Controller drivePID;
    PID_Controller strafePID;
    PID_Controller headingPID;

    public BotToPoint_Control(String flName, String frName, String brName, String blName, HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry){
        driveTrain = new MecanumDriveTrain_Old(flName, frName, brName, blName, hardwareMap);

        drivePID = new PID_Controller(0.5);
        strafePID = new PID_Controller(0.5);
        headingPID = new PID_Controller(0.5);

        drivePID.tolerance = 0.5;
        strafePID.tolerance = 0.5;

        driveTrain.setBreakMode();
        driveTrain.reset();

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        DriveState = driveState.MANUAL;
    }

    public void Drive(){
        driveTrain.update();

        switch (DriveState){
            case MANUAL:
                drive = gamepad1.left_stick_y; //Between -1 and 1
                turn = gamepad1.right_stick_x;
                strafe = gamepad1.left_stick_x;

                if (gamepad1.x){
                    DriveState = driveState.TO_POINT;
                }

                break;

            case TO_POINT:
                if (gamepad1.y){
                    DriveState = driveState.MANUAL;
                }

                drive = drivePID.PID_Power(driveTrain.getPoseEstimate().getX(), -10);
                strafe = strafePID.PID_Power(driveTrain.getPoseEstimate().getY(), 10);
                turn = -headingPID.PID_Power(driveTrain.getPoseEstimate().getHeading(), 0);
        }


        // Mecanum Drive Calculations
        fLeft = 0.875 * drive - 1 * strafe - 0.8 * turn;
        fRight = 0.875 * drive + 1 * strafe + 0.8 * turn;
        bRight = 0.875 * drive - 1 * strafe + 0.8 * turn;
        bLeft = 0.875 * drive + 1 * strafe - 0.8 * turn;

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

    public void Telemetry(){
        // Print pose to telemetry
        telemetry.addData("x", driveTrain.getPoseEstimate().getX());
        telemetry.addData("y", driveTrain.getPoseEstimate().getY());
        telemetry.addData("heading", driveTrain.getPoseEstimate().getHeading());
        telemetry.update();
    }
}
