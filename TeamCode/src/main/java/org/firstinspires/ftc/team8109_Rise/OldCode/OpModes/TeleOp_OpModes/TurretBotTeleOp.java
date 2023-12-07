package org.firstinspires.ftc.team8109_Rise.OldCode.OpModes.TeleOp_OpModes;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team8109_Rise.OldCode.Control.TeleOp.TurretBotTeleOp_Control;

@TeleOp
@Disabled
public class TurretBotTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        TurretBotTeleOp_Control turret = new TurretBotTeleOp_Control("frontLeft", "frontRight", "backRight", "backLeft","spinny", hardwareMap, gamepad1, telemetry);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();

            turret.ControlHubColor();
            turret.teleOpDrive();
            turret.turretSpin();
            turret.Telemetry();

            telemetry.update();
        }
        turret.closeCamera();
    }
}
