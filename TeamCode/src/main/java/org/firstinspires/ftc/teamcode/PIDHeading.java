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
import org.firstinspires.ftc.teamcode.PIDConstants.PIDControlAngleWrap;

@Autonomous(name ="PID Heading")
public class PIDHeading extends LinearOpMode {
    Hardware robot;
    HardwareDriveBase TestDriveBase;
    ElapsedTime runtime = new ElapsedTime();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    PIDControlAngleWrap turnPidController;

    public void runOpMode() throws InterruptedException {
        robot = new Hardware(hardwareMap,telemetry);
        TestDriveBase = new HardwareDriveBase(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turnPidController = new PIDControlAngleWrap();
        turnPidController.setOutputLimit(.5);
        TestDriveBase.leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TestDriveBase.rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();


        while (opModeIsActive()) {
            double referenceAngle = Math.toRadians(PIDConstantsHeading.referenceAngle);
            double power = turnPidController.PIDControl(referenceAngle, TestDriveBase.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
            power(power);

        }
    }
    public void power(double output){
        TestDriveBase.leftRearWheel.setPower(-output);
        TestDriveBase.rightRearWheel.setPower(output);
    }
}
