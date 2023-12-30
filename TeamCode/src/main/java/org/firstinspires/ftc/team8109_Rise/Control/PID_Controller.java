package org.firstinspires.ftc.team8109_Rise.Control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID_Controller {

    public ElapsedTime runtime = new ElapsedTime();

    public double tolerance;

    public double kp;
    public double kd;
    public double ki;
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

    public PID_Controller(double kp){
        this(kp, 0, 0, 0);
    }

    public PID_Controller(double kp, double kd){
        this(kp, kd, 0, 0);
    }

    public PID_Controller(double kp, double kd, double a){
        this(kp, kd, a, 0);
    }

    public PID_Controller(double kp, double kd, double a, double ki){
        this.kp = kp;
        this.kd = kd;
        this.a = a;
        this.ki = ki;
    }

    public double PID_Power(double currPos, double targetPos){
        error = targetPos - currPos;
        errorChange = error - previousError;

        P = kp*error;

        deltaTime = runtime.seconds();
        runtime.reset();

        area += ((error+previousError)*deltaTime)/2;

        if (Math.abs(error) < tolerance){
            area = 0;
        }

        if (targetPos != previousTarget) area = 0;

        I = area*ki;

        currentFilterEstimate = (1-a) * errorChange + (a * previousFilterEstimate);

        D = kd * (currentFilterEstimate / deltaTime);

        previousError = error;
        previousFilterEstimate = currentFilterEstimate;
        previousTarget = targetPos;

        return P + I + D;
    }


    public double PID_PowerBasic(double currPos, double targetPos){
        error = targetPos - currPos;

        errorChange = error - previousError;

        P = kp*error;

        deltaTime = runtime.seconds();
        runtime.reset();

        area += error*deltaTime;

        I = area*ki;

        D = kd * (errorChange / deltaTime);

        previousError = error;
        previousFilterEstimate = currentFilterEstimate;
        previousTarget = targetPos;

        return P + I + D;
    }


    public void setPIDCoefficients(double kp){
        this.kp = kp;
    }

    public void setPIDCoefficients(double kp, double kd){
        this.kp = kp;
        this.kd = kd;
    }

    public void setPIDCoefficients(double kp, double kd, double a){
        this.kp = kp;
        this.kd = kd;
        this.a = a;
    }

    public void setPIDCoefficients(double kp, double kd, double a, double ki){
        this.kp = kp;
        this.kd = kd;
        this.a = a;
        this.ki = ki;
    }
}
