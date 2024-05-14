package org.firstinspires.ftc.teamcode.PIDConstants;

import com.acmerobotics.dashboard.config.Config;


@Config
public class PIDConstantsHeading {
    public static double Kp = 10;  // Proportional gain
    public static double Ki = 0.0;   // Integral gain ( KEEP THIS 0 )
    public static double Kd = .3;   // Derivative gain
    public static double referenceAngle = 0; //Angle

}