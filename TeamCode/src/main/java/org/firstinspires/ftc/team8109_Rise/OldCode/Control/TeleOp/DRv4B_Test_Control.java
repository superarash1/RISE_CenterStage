package org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Control.PIDF_Controller;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.DoubleReverse4Bar_Old;
import org.firstinspires.ftc.team8109_Rise.OldCode.Hardware.MecanumDriveTrain_Old;

public class DRv4B_Test_Control {
    public DoubleReverse4Bar_Old DRv4B_Left;
    public DoubleReverse4Bar_Old DRv4B_Right;

    MecanumDriveTrain_Old driveTrain;

//    public Claw servoClaw;
    public enum barState{
        HOME,
        HIGH_LEVEL
    }

    double drive;
    double turn;
    double strafe;
    double fLeft;
    double fRight;
    double bLeft;
    double bRight;
    double max;

    boolean barToggle1 = true;
    boolean barToggle2 = false;

    double targetHeight;

    boolean lastToggleX = false;

    double drv4b_power = 0;
///
    public Telemetry telemetry;
    public Gamepad gamepad1;
    public PIDF_Controller Drv4B_PID;

    barState BarState;

    public DRv4B_Test_Control(String BarMotorLeftName, String BarMotorRightName, Telemetry telemetry, Gamepad gamepad1, HardwareMap hardwareMap){
        // 1.5"
        DRv4B_Left = new DoubleReverse4Bar_Old(BarMotorLeftName, 0.2, 0.25, 12.3, 39, 12.5, -30.1, hardwareMap);//0.3
        DRv4B_Right = new DoubleReverse4Bar_Old(BarMotorRightName, 0.2, 0.25, 12.3, 39, 13.5, -30.1, hardwareMap);
        //0.35

        driveTrain = new MecanumDriveTrain_Old("fLeft", "fRight", "bRight", "bLeft", hardwareMap);

        driveTrain.setBreakMode();
        driveTrain.reset();

//        servoClaw = new Claw("claw", hardwareMap);

        DRv4B_Left.barMotor.reset();
        DRv4B_Right.barMotor.reset();

        DRv4B_Right.barMotor.setDirectionReverse();

//        Drv4B_PID = new PIDF_Controller(0.6, 0.00004, 0, 0.0025); //0.3
        Drv4B_PID = new PIDF_Controller(0.015, 0.0000175, 0, 0.00075); //0.5, 0.00001
        Drv4B_PID.tolerance = 2;

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        BarState = barState.HOME;
    }

    public void DoubleReverse4Bar(){
        drv4b_power = Drv4B_PID.PIDF_Power(DRv4B_Left.getHeight(), targetHeight);
        switch (BarState){
            case HOME:
                if ((gamepad1.x != lastToggleX) && gamepad1.x && barToggle1){
                    barToggle1 = false;
                    barToggle2 = true;

                    BarState = barState.HIGH_LEVEL;
                }
                targetHeight = 4.5;
                break;
            case HIGH_LEVEL:
                if ((gamepad1.x != lastToggleX) && gamepad1.x && barToggle2){
                    barToggle2 = false;
                    barToggle1 = true;

                    BarState = barState.HOME;
                }
                targetHeight = 37;
                break;
        }

        lastToggleX = gamepad1.x;
        DRv4B_Left.setPower(drv4b_power);
        DRv4B_Right.setPower(drv4b_power);
    }

    public void Drive(){
//        drive = Math.pow(gamepad1.left_stick_y, 3); //Between -1 and 1
//        turn = Math.pow(gamepad1.right_stick_x, 3);
//        strafe = Math.pow(gamepad1.left_stick_x, 3);

        drive = gamepad1.left_stick_y; //Between -1 and 1
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

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
//        telemetry.addData("Claw Position", servoClaw.clawPosition());
        telemetry.addData("PID Power", drv4b_power);
        telemetry.addData("Integral Power", Drv4B_PID.D);
        telemetry.addData("Left Bar Position", DRv4B_Left.getHeight());
        telemetry.addData("Right Bar Position", DRv4B_Right.getHeight());
        telemetry.addData("Angle", Math.toDegrees(DRv4B_Right.getAngle()));
        telemetry.addData("Anti Gravity", DRv4B_Left.antiGravity);
        telemetry.addData("Total Power", DRv4B_Left.antiGravity + drv4b_power);
        telemetry.addData("Target Position", targetHeight);
        telemetry.update();
    }
}
