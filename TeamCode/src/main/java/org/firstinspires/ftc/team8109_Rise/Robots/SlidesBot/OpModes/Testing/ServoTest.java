package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.Mechanisms.ServoIntakeArm;

@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ServoIntakeArm arm = new ServoIntakeArm(gamepad1, telemetry, hardwareMap);
//        Servo armServo1 = hardwareMap.get(Servo.class, "armRight");
//        Servo armServo2 = hardwareMap.get(Servo.class, "armRight");
//        Claw claw = new Claw(gamepad1, telemetry, hardwareMap);
//        Wrist wrist = new Wrist(gamepad1, hardwareMap);

//        OdoRetract retract = new OdoRetract(gamepad1, hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            arm.togglePosition();
//            arm.setAngleIndividual();
            arm.setTelemetry();
            telemetry.update();
//            retract.toggleState();
//            retract.setPodPosition();
//            wrist.setPosition();

//            claw.toggleClaw();
//            claw.setPosition();
//            claw.setTelemetry();


            //TODO: revermosempm

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
