package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.PIDConstants.PIDConstantsHeading;

@Autonomous(name ="PID Heading")
public class PIDHeading extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    double integralSum = 0;
    double Kp = PIDConstantsHeading.Kp;
    double Ki = PIDConstantsHeading.Ki;
    double Kd = PIDConstantsHeading.Kd;
    private double lastError = 0;
    double referenceAngle = Math.toRadians(PIDConstantsHeading.referenceAngle);
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            double power = PIDControl(referenceAngle, robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
            power(power);
        }

    }
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
    public void power(double output){
        robot.leftRearWheel.setPower(-output);
        robot.rightRearWheel.setPower(output);
    }
}
