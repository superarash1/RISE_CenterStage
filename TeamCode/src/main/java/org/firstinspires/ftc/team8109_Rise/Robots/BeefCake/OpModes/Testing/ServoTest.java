package org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Mechanisms.ClawLeft;
import org.firstinspires.ftc.team8109_Rise.Robots.BeefCake.Mechanisms.ClawRight;

@TeleOp
public class ServoTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        //        Servo armServo1 = hardwareMap.get(Servo.class, "armRight");
//        Servo armServo2 = hardwareMap.get(Servo.class, "armRight");
        ClawLeft clawLeft = new ClawLeft(gamepad1, telemetry, hardwareMap);
        ClawRight clawRight = new ClawRight(gamepad1, telemetry, hardwareMap);
//        Wrist wrist = new Wrist(gamepad1, hardwareMap);

//        OdoRetract retract = new OdoRetract(gamepad1, hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.update();
//            retract.toggleState();
//            retract.setPodPosition();
//            wrist.setPosition();

//            claw.toggleClaw();
//            clawLeft.setPosition();
//            clawRight.setPosition();

            clawLeft.toggleClaw();
            clawRight.toggleClaw();

            //            claw.setTelemetry();


            //TODO: rever                                              mosempm

//            telemetry.addData("leftServoPos", armServo1.getPosition());
////            telemetry.addData("rightServoPos", armServo2.getPosition());
//            telemetry.update();
//
//            if (gamepad1.a){
////                arm.armServo1.setPosition(60);
//                setAngle(armServo1, 140);
//            }
//
//            if (gamepad1.b){
//                setAngle(armServo1, 180);
//            }

//
//            telemetry.addData("podState", retract.podState);
//            telemetry.update();
        }
    }

    public void setAngle(Servo servo, double angle){
        servo.setPosition(angle/300);
    }
}
