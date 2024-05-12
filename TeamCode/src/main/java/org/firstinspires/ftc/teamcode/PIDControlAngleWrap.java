package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDConstants.PIDConstantsHeading;

public class PIDControlAngleWrap {

    private ElapsedTime runtime = new ElapsedTime();
    double integralSum = 0;
    double Kp = PIDConstantsHeading.Kp;
    double Ki = PIDConstantsHeading.Ki;
    double Kd = PIDConstantsHeading.Kd;
    private double lastError = 0;

    public double PIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        integralSum += error * runtime.seconds();
        double derivative = (error - lastError) / (runtime.seconds());
        lastError = error;
        runtime.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }


}
