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
    PIDControlAngleWrap turnPidController;

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turnPidController = new PIDControlAngleWrap();
        turnPidController.setOutputLimit(.5);
        robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            double referenceAngle = Math.toRadians(PIDConstantsHeading.referenceAngle);
            double power = turnPidController.PIDControl(referenceAngle, robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
            power(power);
        }

    }
    public void power(double output){
        robot.leftRearWheel.setPower(-output);
        robot.rightRearWheel.setPower(output);
    }
}
