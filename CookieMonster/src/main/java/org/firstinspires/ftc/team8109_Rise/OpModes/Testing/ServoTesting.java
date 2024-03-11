package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team8109_Rise.Mechanisms.CookieMonster_Arm;

//@TeleOp
public class ServoTesting extends LinearOpMode {

    boolean toggle1 = true;
    boolean toggle2 = false;
    boolean lastToggleX = false;
    double position = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        CookieMonster_Arm arm = new CookieMonster_Arm(gamepad1, telemetry, hardwareMap);
//        CookieMonster_Gate gate = new CookieMonster_Gate(gamepad1, telemetry, hardwareMap);

        telemetry.addLine("Waiting for Start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()){
//            arm.Cage.setPosition(0.8);
//            gate.toggleGate();
//            gate.gate.setPosition(0);

            arm.Cage.setPosition(position);
            // Test positions manually
//            if ((gamepad1.x != lastToggleX) && gamepad1.x && toggle1){
//                toggle1 = false;
//                toggle2 = true;
//
//                position = 1;
//            }
//            if ((gamepad1.x != lastToggleX) && gamepad1.x && toggle2){
//                toggle2 = false;
//                toggle1 = true;
//
//                position = 0;
//            }
//            lastToggleX = gamepad1.x;

            telemetry.addData("position", position);
            telemetry.update();
        }
    }
}
