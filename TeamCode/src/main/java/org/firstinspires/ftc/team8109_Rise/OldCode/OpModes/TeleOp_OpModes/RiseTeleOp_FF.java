package org.firstinspires.ftc.team8109_Rise.OldCode.OpModes.TeleOp_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp.RiseTeleOp_FF_Control;

@TeleOp
@Disabled
public class RiseTeleOp_FF extends LinearOpMode {
    @Override
    public void runOpMode() {

        RiseTeleOp_FF_Control teleOp = new RiseTeleOp_FF_Control("frontLeft", "frontRight", "backRight", "backLeft", "armLeft",  "armRight", "cage", "gate", "intake", hardwareMap, telemetry, gamepad1);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            teleOp.teleOpDrive();
            teleOp.Arm();
            teleOp.cageRotation();
            teleOp.teleOpGate();
            teleOp.teleOpIntake();
            teleOp.Telemetry();
        }
    }
};