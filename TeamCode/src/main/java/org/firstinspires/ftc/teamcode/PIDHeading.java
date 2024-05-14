package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.PIDConstants.PIDConstantsHeading;

@Autonomous(name ="PID Heading")
public class PIDHeading extends LinearOpMode {
    Hardware robot;
    public DcMotor leftRearWheel;
    public DcMotor rightRearWheel;
    ElapsedTime runtime = new ElapsedTime();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    PIDControlAngleWrap turnPidController;
    private BHI260IMU imu;

    public PIDHeading(HardwareMap hwMap) {
        leftRearWheel = hwMap.get(DcMotor.class, "CM3");
        leftRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearWheel.setPower(0);

        rightRearWheel = hwMap.get(DcMotor.class, "CM0");
        rightRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearWheel.setDirection(DcMotor.Direction.FORWARD);
        rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearWheel.setPower(0);

        imu = hwMap.get(BHI260IMU.class,"imu");
        IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        turnPidController = new PIDControlAngleWrap();
        turnPidController.setOutputLimit(0.5);
    }

    public void runOpMode() throws InterruptedException {
        robot = new Hardware(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turnPidController = new PIDControlAngleWrap();
        turnPidController.setOutputLimit(.5);
        leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            double referenceAngle = Math.toRadians(PIDConstantsHeading.referenceAngle);
            double power = turnPidController.PIDControl(referenceAngle, imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
            power(power);

        }
    }
    public void power(double output){
        leftRearWheel.setPower(-output);
        rightRearWheel.setPower(output);
    }
}
