package org.firstinspires.ftc.teamcode.PIDConstants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

@Config
public class PIDConstantsHeading {
    public static double Kp = 6;  // Proportional gain
    public static double Ki = 0.0;   // Integral gain ( KEEP THIS 0 )
    public static double Kd = 5.5;   // Derivative gain
    public static double referenceAngle = 0; //Angle

    @Autonomous(name ="PID Drive")
    public static class PIDDrive extends LinearOpMode {

        ElapsedTime runtime = new ElapsedTime();
        Hardware robot = Hardware.getInstance();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        double integralSum = 0;
        double Kp = PIDConstantsDrive. Kp;
        double Ki = PIDConstantsDrive.Ki;
        double Kd = PIDConstantsDrive.Kd;
        private double lastError = 0;
        double distanceInches = PIDConstantsDrive.distanceInches;
        double countsPerInch = 384.5 / (4 * Math.PI);
        double referenceDistance = distanceInches * countsPerInch;
        public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            waitForStart();

            while (opModeIsActive()) {
                double power = PIDControl(referenceDistance, robot.leftRearWheel.getCurrentPosition());
                power(power);
            }

        }
        public double PIDControl(double refrence, double state) {
            double error = refrence - state;
            integralSum += error * runtime.seconds();
            double derivative = (error - lastError) / (runtime.seconds());
            lastError = error;
            runtime.reset();
            double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
            return output;
        }
        public void power(double output){
            robot.leftRearWheel.setPower(output);
            robot.rightRearWheel.setPower(output);
        }
    }
}
