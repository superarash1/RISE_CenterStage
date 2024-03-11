package org.firstinspires.ftc.robotcontroller.Control.MotionProfiling;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TrapezoidalMotionProfile {

    public double max_acceleration;
    public double max_velocity;

    public boolean acceleration_dt_test = false;
    public double current_dt = 0;
    public double entire_dt = 0;
    public double cruise_current_dt = 0;
    public double acceleration_dt = 0;
    public double deacceleration_dt = 0;
    public double cruise_dt = 0;

    public double distance = 0;
    public double halfway_distance = 0;
    public double acceleration_distance = 0.1;
    public double cruise_distance = 0;

    public double deacceleration_time = 0;

    public double previousTargetPos = 0;

    public double x = 0;
    public double v = 0;
    public double a = 0;

    public double positionError = 0;
    public double error = 0;

    public double trajectoryPos = 0;
    public double initialPos = 0;

    public double kp;
    public double kv;
    public double ka;

    public double tolerance;

    public enum profileState{
        ACCELERATING,
        CRUISE,
        DECELERATING,
        PAST_SETPOINT
    }
    ElapsedTime runtime = new ElapsedTime();

    public profileState ProfileState;
    public TrapezoidalMotionProfile(double max_velocity, double max_acceleration, double kp, double kv, double ka){
        this.max_velocity = max_velocity;
        this.max_acceleration = max_acceleration;

        this.kp = kp;
        this.kv = kv;
        this.ka = ka;

        acceleration_dt = max_velocity / max_acceleration;

        ProfileState = profileState.ACCELERATING;
        runtime.reset();
    }

    // Distance and target pos are not the same
    // TODO: add current dt (time from the beginning of trajectory
    public double getProfilePower(double currPos, double targetPos){
        error = targetPos - currPos;

        if (previousTargetPos != targetPos){

            distance = error;
            initialPos = currPos;

            initialCalculations();
            runtime.reset();
        }
        current_dt = runtime.seconds();

        setProfileState();

        x = motion_profile_position(distance);
        v = motion_profile_velocity();
        a = motion_profile_acceleration();

        trajectoryPos = currPos - initialPos;
        positionError = x - trajectoryPos;

        previousTargetPos = targetPos;

        return positionError*kp + v*kv + a*ka;
    }

//    public void initialCalculations(){
//        current_dt = runtime.seconds();
//
//        // calculate the time it takes to accelerate to max velocity
//        acceleration_dt = max_velocity / max_acceleration;
//
//        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
//        halfway_distance = distance / 2;
//        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);
//
//        if (acceleration_distance > Math.abs(halfway_distance)){
//            acceleration_dt = Math.sqrt(Math.abs(halfway_distance) / (0.5 * max_acceleration));
//            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);
//
//            if (acceleration_dt == 0){
//                acceleration_dt_test = true;
//            }
//            // recalculate max velocity based on the time we have to accelerate and decelerate
//            max_velocity = max_acceleration * acceleration_dt;
//        }
//
//        // we decelerate at the same rate as we accelerate
//        deacceleration_dt = acceleration_dt;
//
//        // calculate the time that we're at max velocity
//        cruise_distance = distance - 2 * acceleration_distance;
//        cruise_dt = cruise_distance / max_velocity;
//        deacceleration_time = acceleration_dt + cruise_dt;
//
//        // check if we're still in the motion profile
//        entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
//    }

    public void initialCalculations(){


        // calculate the time it takes to accelerate to max velocity
        acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        halfway_distance = distance / 2;
        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);

        if (acceleration_distance > Math.abs(halfway_distance)){
            acceleration_dt = Math.sqrt(Math.abs(halfway_distance) / (0.5 * max_acceleration));
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);

            if (acceleration_dt == 0){
                acceleration_dt_test = true;
            }
            // recalculate max velocity based on the time we have to accelerate and decelerate
            max_velocity = max_acceleration * acceleration_dt;
        }

        // we decelerate at the same rate as we accelerate
        deacceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        cruise_distance = distance - 2 * acceleration_distance;
        cruise_dt = cruise_distance / max_velocity;
        deacceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
    }

    public void setProfileState(){
        if (current_dt > entire_dt){
            ProfileState = profileState.PAST_SETPOINT;
        } else if (current_dt < acceleration_dt){
            ProfileState = profileState.ACCELERATING;
        } else if (current_dt < deacceleration_time) {
            ProfileState = profileState.CRUISE;
        } else {
            ProfileState = profileState.DECELERATING;
        }
    }

    public double motion_profile_position(double distance) {
        switch (ProfileState){
            case ACCELERATING:
                return 0.5 * max_acceleration * Math.pow(current_dt, 2);
            case CRUISE:
                // if we're cruising
                acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);
                cruise_current_dt = current_dt - acceleration_dt;

                // use the kinematic equation for constant velocity
                return acceleration_distance + max_velocity * cruise_current_dt;
            case DECELERATING:
                // if we're decelerating
                acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);
                cruise_distance = max_velocity * cruise_dt;

                // use the kinematic equations to calculate the instantaneous desired position   x = 1/2at^2 +v0t + x0
                return acceleration_distance + cruise_distance + max_velocity * (current_dt - deacceleration_time) - 0.5 * max_acceleration * Math.pow(acceleration_dt,2);
            default:
                return distance;
        }
    }

    public double motion_profile_velocity(){
        switch (ProfileState) {
            case ACCELERATING:
                return max_acceleration * current_dt;
            case CRUISE:
                return max_velocity;
            case DECELERATING:
                // use the kinematic equations to calculate the instantaneous desired velocity  v = v0 + at
                return max_velocity - max_acceleration * (current_dt - deacceleration_time);
            default:
                return 0;
        }
    }

    public double motion_profile_acceleration(){
        switch (ProfileState) {
            case ACCELERATING:
                return max_acceleration;
            case DECELERATING:
                return -max_acceleration;
            default:
                return 0;
        }
    }

    public void setMaxVel(double max_velocity){
        this.max_velocity = max_velocity;
    }

    public void setMaxAccel(double max_acceleration){
        this.max_acceleration = max_acceleration;
    }
}