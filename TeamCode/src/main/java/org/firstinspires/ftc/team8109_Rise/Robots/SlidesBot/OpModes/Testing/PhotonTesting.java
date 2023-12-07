package org.firstinspires.ftc.team8109_Rise.Robots.SlidesBot.OpModes.Testing;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PhotonTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        PhotonCore.enable();

        waitForStart();
        double lastTime = 0;
        while (opModeIsActive()){
            telemetry.addData("looptimes",runtime.seconds());
            runtime.reset();
        }
    }
}
