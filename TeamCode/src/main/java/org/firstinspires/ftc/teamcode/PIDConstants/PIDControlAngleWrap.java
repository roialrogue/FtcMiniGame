package org.firstinspires.ftc.teamcode.PIDConstants;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PIDConstants.PIDConstantsHeading;

public class PIDControlAngleWrap {
    private ElapsedTime runtime = new ElapsedTime();
    double integralSum = 0;
    private double lastError = 0;
    double outputLimit = 1;

    public double PIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        integralSum += error * runtime.seconds();
        double derivative = (error - lastError) / (runtime.seconds());
        lastError = error;
        runtime.reset();
        double output = (error * PIDConstantsHeading.Kp) + (derivative * PIDConstantsHeading.Kd) + (integralSum * PIDConstantsHeading.Ki);
        return Range.clip(output,-outputLimit,outputLimit);
    }

    private double angleWrap(double degrees){
        while(degrees > 180.0){
            degrees -= 360.0;
        }
        while(degrees < -180.0){
            degrees += 360.0;
        }
        return degrees;
    }

    public void setOutputLimit(double limit) {
        outputLimit = limit;
    }
}