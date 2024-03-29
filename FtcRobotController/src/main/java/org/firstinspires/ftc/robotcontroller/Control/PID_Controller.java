package org.firstinspires.ftc.robotcontroller.Control;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID_Controller {

    public ElapsedTime runtime = new ElapsedTime();

    public double tolerance;

    public PIDCoefficients coeffs;
    public double a;

    public double error;

    public double area;

    public double P = 0;
    public double I = 0;
    public double D = 0;

    public double deltaTime;
    public double previousError = 0;
    public double previousTarget = 0;

    public double previousFilterEstimate = 0;
    public double currentFilterEstimate = 0;

    public double errorChange;

    @Deprecated
    public PID_Controller(double p) {
        this.coeffs = new PIDCoefficients(p, 0, 0);
        this.a = 0;
    }
    @Deprecated
    public PID_Controller(double p, double d, double i) {
        this.coeffs = new PIDCoefficients(p, i, d);
        this.a = 0;
    }
    @Deprecated
    public PID_Controller(double p, double d, double i, double a) {
        this.coeffs = new PIDCoefficients(p, i, d);
        this.a = a;
    }
    public PID_Controller(PIDCoefficients coeffs, double a){
        this.coeffs = coeffs;
        this.a = a;
    }

    public double PID_Power(double currPos, double targetPos){
        error = targetPos - currPos;
        errorChange = error - previousError;

        P = coeffs.p*error;

        deltaTime = runtime.seconds();
        runtime.reset();

        area += ((error+previousError)*deltaTime)/2;

        if (Math.abs(error) < tolerance) area = 0;
        if (targetPos != previousTarget) area = 0;

        I = area*coeffs.i;

        currentFilterEstimate = (1-a) * errorChange + (a * previousFilterEstimate);

        D = coeffs.d * (currentFilterEstimate / deltaTime);

        previousError = error;
        previousFilterEstimate = currentFilterEstimate;
        previousTarget = targetPos;

        return P + I + D;
    }


    public double PID_PowerBasic(double currPos, double targetPos){
        error = targetPos - currPos;

        errorChange = error - previousError;

        P = coeffs.p*error;

        deltaTime = runtime.seconds();
        runtime.reset();

        D = coeffs.d * (errorChange / deltaTime);

        area += error*deltaTime;

        I = area*coeffs.i;

        previousError = error;
        previousFilterEstimate = currentFilterEstimate;
        previousTarget = targetPos;

        return P + I + D;
    }


    public void setPIDCoefficients(double kp){
        this.coeffs.p = kp;
    }

    public void setPIDCoefficients(double kp, double kd){
        this.coeffs.p = kp;
        this.coeffs.d = kd;
    }

    public void setPIDCoefficients(double kp, double kd, double a){
        this.coeffs.p = kp;
        this.coeffs.d = kd;
        this.a = a;
    }

    public void setPIDCoefficients(double kp, double kd, double a, double ki){
        this.coeffs.p = kp;
        this.coeffs.d = kd;
        this.a = a;
        this.coeffs.i = ki;
    }
}
