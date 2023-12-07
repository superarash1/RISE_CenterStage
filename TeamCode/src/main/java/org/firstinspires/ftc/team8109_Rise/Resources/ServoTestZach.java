package org.firstinspires.ftc.team8109_Rise.Resources;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Five Turn Servo Test")
@Disabled
public class ServoTestZach extends LinearOpMode {
    Servo fiveturn;

    @Override
    public void runOpMode() {
        fiveturn = hardwareMap.get(Servo.class, "fiveturn");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Status", "Running");
            telemetry.addData("Servo Position", fiveturn.getPosition());
            telemetry.update();

            if (gamepad1.dpad_down) {
                fiveturn.setPosition(0.93);
            } else if (gamepad1.dpad_up) {
                fiveturn.setPosition(0.97);
            } else if (gamepad1.dpad_left) {
                fiveturn.setPosition(1);
            }
        }
    }
}

