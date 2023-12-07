package org.firstinspires.ftc.team8109_Rise.OldCode.OpModes.TeleOp_OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp.DRv4B_Test_Control;


public class Protobot_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DRv4B_Test_Control control = new DRv4B_Test_Control("dr4bLeft", "dr4bRight", telemetry, gamepad1, hardwareMap);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

//        telemetry.clearAll();

        while (opModeIsActive()){
            control.Drive();
            control.DoubleReverse4Bar();
            control.Telemetry();
        }
    }
}
