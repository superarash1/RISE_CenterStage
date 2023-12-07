package org.firstinspires.ftc.team8109_Rise.OldCode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team8109_Rise.Hardware.Motor;

public class Arm {

    public Motor armMotor;

    double startAngle;

    public Arm(String Arm, double startAngle, double gearRatio, HardwareMap hardwareMap){
        armMotor = new Motor(Arm, 537.7,hardwareMap);

        armMotor.reset();

        armMotor.setBreakMode();

        this.startAngle = startAngle;
    }

    public double armAngle(){
        return armMotor.getCurrPosDegrees() + startAngle;
    }

//    public void ArmMotion(){
//
//        this.armLeft.setPower(PIDF.PIDF_Arm(armLeft.getCurrPosDegrees(), targetPos, 0.6,0, 0,0,0.03, firstError));
//        this.armRight.setPower(PIDF.PIDF_Arm(armLeft.getCurrPosDegrees(), targetPos, 0.6,0, 0,0,0.03, firstError));
//
//        switch (ArmState){
//            case HOME:
//                targetPos = -33;
//                firstError = targetPos - armLeft.getCurrPosDegrees();
//
//                CageState = cageState.FLAT;
//                break;
//            case TOP_LEVEL:
//                targetPos = 135;
//
//                firstError = 178.8;
//                CageState = cageState.DROP;
//
//                break;
//            case MIDDLE_LEVEL:
//                targetPos = 175;
//                firstError = targetPos - armLeft.getCurrPosDegrees();
//                CageState = cageState.FLAT;
//
//                break;
//            case BOTTOM_LEVEL:
//                targetPos = 205;
//                firstError = targetPos - armLeft.getCurrPosDegrees();
//                CageState = cageState.FLAT;
//
//                break;
//        }
//
//
////        //This is the working block of code \/\/ (I think)
////        if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle1) {
////            ArmState = armState.TOP_LEVEL;
////            armToggle1 = false;
////            armToggle2 = true;
////
////        } else if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle2) {
////            ArmState = armState.HOME;
////
////            armToggle2 = false;
////            armToggle1 = true;
////        }
////
////        lastToggleX = gamepad1.x;
//
////        if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle1) {
////            targetPos = 135;
////            firstError = 178.8;
////
////            boxTarget = 0;
////
////            armToggle1 = false;
////            armToggle2 = true;
////
////        } else if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle2) {
////            targetPos = -33;
////            firstError = targetPos - armLeft.getCurrPosDegrees();
////
////            boxTarget = 0.7;
////
////            armToggle2 = false;
////            armToggle1 = true;
////        }
////
////        lastToggleX = gamepad1.x;
////
////        if (gamepad1.dpad_up && armLeft.getCurrPosDegrees() > 100 && armLeft.getCurrPosDegrees() < 160 && armCoolDown > 100){
////            targetPos = 175;
////            firstError = targetPos - armLeft.getCurrPosDegrees();
////            boxTarget = 0;
////
////            armCoolDown = 0;
////        }
////
////        if (gamepad1.dpad_up && armLeft.getCurrPosDegrees() > 160 && armCoolDown > 100){
////            targetPos = 205;
////            firstError = targetPos - armLeft.getCurrPosDegrees();
////            boxTarget = 0;
////
////            armCoolDown = 0;
////        }
//    }
//
//    public void cageRotation(){
//        cageSpin.setPosition(boxTarget);
//
//        switch (CageState){
//            case FLAT:
//                boxTarget = 0;
//
//                break;
//            case DROP:
//                boxTarget = 0.7;
//
//                break;
//            case MANUAL:
////                if (gamepad1.right_bumper) {
////                    cageSpin.setPosition(boxTarget - 0.01);
////                } else if (gamepad1.left_bumper) {
////                    cageSpin.setPosition(boxTarget + 0.01);
////                }
//                break;
//        }
//    }
}
