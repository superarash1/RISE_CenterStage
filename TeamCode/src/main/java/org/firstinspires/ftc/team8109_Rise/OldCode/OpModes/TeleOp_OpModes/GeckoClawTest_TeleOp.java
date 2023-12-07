package org.firstinspires.ftc.team8109_Rise.OldCode.OpModes.TeleOp_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp.GeckoClawTestControl;

@TeleOp
@Disabled
public class GeckoClawTest_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GeckoClawTestControl control = new GeckoClawTestControl(hardwareMap, gamepad1, telemetry);

        telemetry.addLine("Waiting for start");
        waitForStart();

        while (opModeIsActive()){
            control.Claw();
            control.Telemetry();
        }
    }
}
