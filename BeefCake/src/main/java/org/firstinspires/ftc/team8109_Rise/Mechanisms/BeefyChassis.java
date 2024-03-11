package org.firstinspires.ftc.team8109_Rise.Mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.Control.MotionProfiling.TrapezoidalMotionProfile;
import org.firstinspires.ftc.robotcontroller.Control.PID_Controller;
import org.firstinspires.ftc.robotcontroller.Hardware.Drivetrains.MecanumDriveTrain;
import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team8109_Rise.Sensors.Odometry.BeefCake_OdometryLocalizer;
import org.firstinspires.ftc.team8109_Rise.Sensors.Odometry.BeefyDriveConstants;

public class BeefyChassis extends MecanumDriveTrain {

    public TrapezoidalMotionProfile velocityController;

    public BeefyChassis(Gamepad gamepad1, Telemetry telemetry, HardwareMap hardwareMap){
        super("fLeft", "fRight", "bRight", "bLeft", new BeefyDriveConstants(),
                OdometryType.THREE_WHEEL_ODO, new BeefCake_OdometryLocalizer(hardwareMap), hardwareMap);

        reset();

        TranslationalPID_X = new PID_Controller(0, 0, 0, 0);//12.5 volts, a = 0
        TranslationalPID_Y = new PID_Controller(0, 0, 0, 0);
        HeadingPID = new PID_Controller(0, 0, 0, 0);

        velocityController = new TrapezoidalMotionProfile(CONSTANTS.MAX_VEL, CONSTANTS.MAX_ACCEL, 0, 0, 0);

        TranslationalProfile_X = new TrapezoidalMotionProfile(CONSTANTS.MAX_VEL, CONSTANTS.MAX_ACCEL, 0, 0, 0);
        TranslationalProfile_Y = new TrapezoidalMotionProfile(CONSTANTS.MAX_VEL, CONSTANTS.MAX_ACCEL, 0, 0, 0);

        TranslationalPID_X.tolerance = 0.05;
        TranslationalPID_Y.tolerance = 0.05;

        TranslationalProfile_X.tolerance = 0.25;
        TranslationalProfile_Y.tolerance = 0.25;

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void setDriveVectorsFieldCentric(Vector3D input){
        x_rotated = input.A * Math.cos(getPoseEstimate().getHeading()) + input.B * Math.sin(getPoseEstimate().getHeading());
        y_rotated = input.A * Math.sin(getPoseEstimate().getHeading()) - input.B * Math.cos(getPoseEstimate().getHeading());

        fLeft = CONSTANTS.VX_WEIGHT * x_rotated + CONSTANTS.VY_WEIGHT * y_rotated - CONSTANTS.OMEGA_WEIGHT * input.C;
        fRight = CONSTANTS.VX_WEIGHT * x_rotated - CONSTANTS.VY_WEIGHT * y_rotated + CONSTANTS.OMEGA_WEIGHT * input.C;
        bRight = CONSTANTS.VX_WEIGHT * x_rotated + CONSTANTS.VY_WEIGHT * y_rotated + CONSTANTS.OMEGA_WEIGHT * input.C;
        bLeft = CONSTANTS.VX_WEIGHT * x_rotated - CONSTANTS.VY_WEIGHT * y_rotated - CONSTANTS.OMEGA_WEIGHT * input.C;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        if (fLeft != previousFLeft) frontLeft.setPower(fLeft);
        if (fRight != previousFRight) frontRight.setPower(fRight);
        if (bRight != previousBRight) backRight.setPower(bRight);
        if (bLeft != previousBLeft) backLeft.setPower(bLeft);

        setPower(fLeft, fRight, bRight, bLeft);

        previousFLeft = fLeft;
        previousFRight = fRight;
        previousBRight = bRight;
        previousBLeft = bLeft;
    }

    public void goToPosePID(Vector3D input){
        odoDrive = -TranslationalPID_X.PID_Power(getPoseEstimate().getX(), input.A);
        odoStrafe = -TranslationalPID_Y.PID_Power(getPoseEstimate().getY(), input.B);
        odoTurn = -HeadingPID.PID_Power(angleWrap(getPoseEstimate().getHeading()), input.C);

        odoPID_Vector.set(odoDrive, odoStrafe, odoTurn);

        setDriveVectorsFieldCentric(odoPID_Vector);
    }

    // time dependent vector valued function and make some kinda object to store a function so we can pass it in as a parameter
    public void vectorValuedFunction(double t){
        //define these in actual auton class?
        //potentially write up a code that can automatically calculate the derivative
        Vector3D velocity = new Vector3D(0,0,0);
        velocity.A = 3*t;
        velocity.B = 7-t*t;
        velocity.C = Math.sin(t);
    }

    // All parameters in this method are vector valued functions (potentially bake parametric equations into the vector class Vector3D.t global  variable for instance)
    public void followParametricEquation(Vector3D velocity, Vector3D idealPosition, double currentArcLength, double totalArcLength){
        double speed = velocityController.getProfilePower(currentArcLength, totalArcLength);
        Vector3D velocityNormalized = velocity.normalize();
        Vector3D idealOutput = velocityNormalized.scale(speed);

        // fix up field centric code to remove negative signs/check if motor directions are correct (using robot centric signs to check if they're correct)
        double correction_X = -TranslationalPID_X.PID_Power(getPoseEstimate().getX(), idealPosition.A);
        double correction_Y = -TranslationalPID_Y.PID_Power(getPoseEstimate().getY(), idealPosition.B);
        double correction_Heading = -HeadingPID.PID_Power(angleWrap(getPoseEstimate().getHeading()), idealPosition.C);
        Vector3D adjustedOutput = idealOutput.add(new Vector3D(correction_X, correction_Y, correction_Heading));
        setDriveVectorsRobotCentric(adjustedOutput);
    }

    public void goToPoseTrapezoidal(Vector3D input){
        odoDrive = -TranslationalProfile_X.getProfilePower(getPoseEstimate().getX(), input.A);
        odoStrafe = -TranslationalProfile_Y.getProfilePower(getPoseEstimate().getY(), input.B);
        odoTurn = -HeadingPID.PID_Power(angleWrap(getPoseEstimate().getHeading()), input.C);

        odoPID_Vector.set(odoDrive, odoStrafe, odoTurn);

        trapezoidalTranslationalError = input.getVector2D().findDistance(getPoseVector().getVector2D());
        setDriveVectorsFieldCentric(odoPID_Vector);
    }


    public void fieldCentricTest(){
        controllerInput.set(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        setDriveVectorsFieldCentric(controllerInput);
    }

    public void chassisTelemetry(){

    }
}
