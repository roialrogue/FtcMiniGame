package org.firstinspires.ftc.teamcode.PIDConstants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
public class PIDConstantsHeading {
    public static double Kp = 6;  // Proportional gain
    public static double Ki = 0.0;   // Integral gain ( KEEP THIS 0 )
    public static double Kd = 5.5;   // Derivative gain
    public static double referenceAngle = 0; //Angle
}
