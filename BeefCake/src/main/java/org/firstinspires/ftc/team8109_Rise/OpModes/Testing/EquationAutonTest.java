package org.firstinspires.ftc.team8109_Rise.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector3D;
import org.firstinspires.ftc.team8109_Rise.Mechanisms.BeefyChassis;

public class EquationAutonTest extends LinearOpMode {
    BeefyChassis chassis;
    ElapsedTime runtime = new ElapsedTime();

    enum Trajectories{
        TRAJECTORY_ONE,
        TRAJECTORY_TWO,
        STOP
    }
    Trajectories trajectories;
    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new BeefyChassis(gamepad1, telemetry, hardwareMap);

        while (opModeInInit()){
            trajectories = Trajectories.TRAJECTORY_ONE;

            telemetry.addLine("Robot is in init; waiting for start");
            telemetry.update();
        }

        while (opModeIsActive()){
            double t = runtime.seconds();
            switch (trajectories){
                case TRAJECTORY_ONE:
                    // actually do math to get real equations
                    chassis.followParametricEquation(new Vector3D(t*t, t,Math.sin(t)), chassis.getPoseVector(), t, t);
                    if (chassis.velocityController.error < 0.5){
                        trajectories = Trajectories.TRAJECTORY_TWO;
                    }
                    break;
                case TRAJECTORY_TWO:
                    // actually do math to get real equations
                    chassis.followParametricEquation(new Vector3D(Math.PI*5*t, t,Math.sin(t)), chassis.getPoseVector(), t, t);
                    if (chassis.velocityController.error < 0.5){
                        trajectories = Trajectories.STOP;
                    }
                    break;
                case STOP:

                    break;
            }
        }
    }
}
