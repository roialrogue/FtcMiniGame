package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveBase
{
    private DcMotor rightRearWheel;
    private DcMotor leftRearWheel;
    private BHI260IMU imu;
    private PIDControlAngleWrap turnPidController;
    private Double turnTarget = null;
    private double currHeading;

    public DriveBase(HardwareMap hwMap)
    {
        leftRearWheel = hwMap.get(DcMotor.class, "CM3");
        leftRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearWheel.setPower(0);
        leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightRearWheel = hwMap.get(DcMotor.class, "CM0");
        rightRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearWheel.setDirection(DcMotor.Direction.FORWARD);
        rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearWheel.setPower(0);
        rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hwMap.get(BHI260IMU.class,"imu");
        IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        turnPidController = new PIDControlAngleWrap();
        turnPidController.setOutputLimit(0.5);
    }

    public void turn(double angle)
    {
        turnTarget = angle;
    }

    public boolean turnOnTarget(double tolerance)
    {
        boolean isOnTarget = false;

        if (turnTarget != null)
        {
            isOnTarget = Math.abs(turnTarget - currHeading) <= tolerance;
            if (isOnTarget)
            {
                turnTarget = null;
            }
        }
        return isOnTarget;
    }

    public void turnTask()
    {
        currHeading = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        if (turnTarget != null)
        {
            double output = turnPidController.PIDControl(currHeading + turnTarget, currHeading);
            leftRearWheel.setPower(-output);
            rightRearWheel.setPower(output);
        }
    }
}
