package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.Bartholomew_Claw;

//@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Bartholomew_Claw claw = new Bartholomew_Claw(gamepad1, telemetry, hardwareMap);
//        Bartholomew_Wrist wrist = new Bartholomew_Wrist(gamepad1,telemetry, hardwareMap);

//        Servo servo = hardwareMap.get(Servo.class, "servo");
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            claw.toggleClaw();

//            wrist.setAngle(0);
            telemetry.addData("clawState", claw.clawState);
            telemetry.update();
        }
    }

    public void setAngle(Servo servo, double angle){
        servo.setPosition(angle/300);
    }
}
