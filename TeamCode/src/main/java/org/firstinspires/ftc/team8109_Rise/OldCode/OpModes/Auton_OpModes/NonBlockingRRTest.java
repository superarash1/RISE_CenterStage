package org.firstinspires.ftc.team8109_Rise.OldCode.OpModes.Auton_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp.Autonomous.RoadRunnerTest_Control;

@Autonomous
@Disabled
public class NonBlockingRRTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        RoadRunnerTest_Control control = new RoadRunnerTest_Control("frontLeft", "frontRight", "backRight", "backLeft", hardwareMap, telemetry);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()){
            control.RR_Drive();
        }
    }
}
