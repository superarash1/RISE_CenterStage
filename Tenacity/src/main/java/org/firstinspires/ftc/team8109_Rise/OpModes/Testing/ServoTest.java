package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.TenacityClaw;

//@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TenacityClaw claw = new TenacityClaw(gamepad1, telemetry, hardwareMap);

//        Servo servo = hardwareMap.get(Servo.class, "servo");
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            claw.toggleClaw();

            telemetry.addData("clawState", claw.clawLeft.clawState);
            telemetry.update();
        }
    }

    public void setAngle(Servo servo, double angle){
        servo.setPosition(angle/300);
    }
}
