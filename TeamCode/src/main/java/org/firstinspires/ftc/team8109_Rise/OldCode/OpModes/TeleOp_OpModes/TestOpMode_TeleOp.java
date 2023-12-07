package org.firstinspires.ftc.team8109_Rise.OldCode.OpModes.TeleOp_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class TestOpMode_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Test Hi");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addLine("Test bye");
            telemetry.update();
        }
    }
}
