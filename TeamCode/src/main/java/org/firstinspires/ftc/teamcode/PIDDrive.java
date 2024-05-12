package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name ="PID Drive")
public class PIDDrive extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    private double lastError = 0;
    double distanceInches = 12;
    double countsPerInch = 384.5 / (4 * Math.PI);
    double referenceDistance = distanceInches * countsPerInch;
    public void runOpMode() throws InterruptedException {
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