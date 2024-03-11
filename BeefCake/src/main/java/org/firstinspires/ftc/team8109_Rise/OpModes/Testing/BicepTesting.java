package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.Bicep;

@TeleOp
public class BicepTesting extends LinearOpMode {

    Bicep bicep;
    @Override
    public void runOpMode() throws InterruptedException {
        bicep = new Bicep(new String[]{"armLeft", "armRight"}, hardwareMap);

        while (opModeInInit()){
            bicep.bicepState = Bicep.BicepStates.HOME;
            telemetry.addLine("Press to Start");
            telemetry.update();
        }

        while (opModeIsActive()){
            bicep.bicepStates();
        }
    }
}
