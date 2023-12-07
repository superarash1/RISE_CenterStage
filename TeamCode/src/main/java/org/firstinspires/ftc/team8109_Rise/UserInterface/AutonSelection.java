package org.firstinspires.ftc.team8109_Rise.UserInterface;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonSelection {
    Telemetry telemetry;
    Gamepad gamepad1;

    public enum AutonSetting {
        LEFT_SIDE_RED,
        RIGHT_SIDE_RED,
        LEFT_SIDE_BLUE,
        RIGHT_SIDE_BLUE
    }

    boolean startToggle = true;
    boolean lastToggleDown = false;
    boolean lastToggleUp = false;

    public AutonSetting autonSetting;

    public AutonSelection (Gamepad gamepad1, Telemetry telemetry){
        autonSetting = AutonSetting.LEFT_SIDE_RED;

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
    }

    public AutonSelection (AutonSetting initialCondition, Gamepad gamepad1, Telemetry telemetry){
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
    }

    public void setAutonMode(){
        while (startToggle){
            telemetry.addLine("Auton Setting:");
            switch (autonSetting){
                case LEFT_SIDE_RED:
                    telemetry.addLine("Left-side Red Auton");

                    if (gamepad1.dpad_down && !lastToggleDown){
                        autonSetting = AutonSetting.RIGHT_SIDE_RED;
                    }
                    break;
                case RIGHT_SIDE_RED:
                    telemetry.addLine("Right-side Red Auton");

                    if (gamepad1.dpad_down && !lastToggleDown){
                        autonSetting = AutonSetting.LEFT_SIDE_BLUE;
                    }

                    if (gamepad1.dpad_up && !lastToggleUp){
                        autonSetting = AutonSetting.LEFT_SIDE_RED;
                    }

                    break;
                case LEFT_SIDE_BLUE:
                    telemetry.addLine("Left-side Blue Auton");

                    if (gamepad1.dpad_down && !lastToggleDown){
                        autonSetting = AutonSetting.RIGHT_SIDE_BLUE;
                    }

                    if (gamepad1.dpad_up && !lastToggleUp){
                        autonSetting = AutonSetting.RIGHT_SIDE_RED;
                    }
                    break;
                case RIGHT_SIDE_BLUE:
                    telemetry.addLine("Right-side Blue Auton");

                    if (gamepad1.dpad_up && !lastToggleUp){
                        autonSetting = AutonSetting.LEFT_SIDE_BLUE;
                    }
                    break;
            }

            lastToggleDown = gamepad1.dpad_down;
            lastToggleUp = gamepad1.dpad_up;

            telemetry.update();

            if (gamepad1.square || gamepad1.x) startToggle = false;
        }
    }
}
