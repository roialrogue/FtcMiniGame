package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDConstants.PIDControlAngleWrap;

public class DriveBase
{
    private static final double countsPerInch = 384.5 / (4 * Math.PI);
    private Telemetry telemetry;
    private ElapsedTime runtime;
    private DcMotor rightRearWheel;
    private DcMotor leftRearWheel;
    private BHI260IMU imu;
    private PIDControlAngleWrap turnPidController;
    private Double absoluteTurnTarget = null;
    private Integer driveTarget = null;
    private double timeout = 0.0;

    public DriveBase(HardwareMap hwMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;
        runtime = new ElapsedTime();
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

    //
    // Drive task.
    //

    public void setDrivePower(double leftPower, double rightPower)
    {
        // Stop PID control if any.
        stopDrive();
        stopTurn();

        leftRearWheel.setPower(leftPower);
        rightRearWheel.setPower(rightPower);
    }

    public void drive(double speed, double distance, double timeout)
    {
        // Stop turn if any.
        stopTurn();

        driveTarget = leftRearWheel.getCurrentPosition() + (int) (distance * countsPerInch);
        this.timeout = timeout;

        leftRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearWheel.setTargetPosition(driveTarget);
        rightRearWheel.setTargetPosition(driveTarget);
        leftRearWheel.setPower(Math.abs(speed));
        rightRearWheel.setPower(Math.abs(speed));

        runtime.reset();
    }

    public void drive(double speed, double distance)
    {
        drive(speed, distance, 0.0);
    }

    public void stopDrive()
    {
        if (driveTarget != null)
        {
            driveTarget = null;
            leftRearWheel.setPower(0);
            rightRearWheel.setPower(0);
            leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public boolean driveOnTarget()
    {
        return !leftRearWheel.isBusy() && !rightRearWheel.isBusy();
    }

    public void driveTask()
    {
        if (driveTarget != null)
        {
            if (timeout > 0.0 && runtime.seconds() >= timeout || driveOnTarget())
            {
                stopDrive();
            }
            telemetry.addData("DriveTask: CurrPos", leftRearWheel.getCurrentPosition() + "," + rightRearWheel.getCurrentPosition());
            telemetry.update();
        }
    }

    //
    // Turn task.
    //

    public double getHeading()
    {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void absoluteTurn(double angle, double timeout)
    {
        // Stop drive if any.
        stopDrive();

        absoluteTurnTarget = angle;
        this.timeout = timeout;
        leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
    }

    public void absoluteTurn(double angle)
    {
        absoluteTurn(angle, 0.0);
    }

    public void relativeTurn(double angle, double timeout)
    {
        absoluteTurn(getHeading() + angle, timeout);
    }

    public void relativeTurn(double angle)
    {
        relativeTurn(angle, 0.0);
    }

    public void stopTurn()
    {
        if (absoluteTurnTarget != null)
        {
            absoluteTurnTarget = null;
            leftRearWheel.setPower(0);
            rightRearWheel.setPower(0);
            leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public boolean turnOnTarget(double tolerance)
    {
        boolean isOnTarget = false;

        if (absoluteTurnTarget != null)
        {
            isOnTarget = timeout > 0.0 && runtime.seconds() >= timeout || Math.abs(absoluteTurnTarget - getHeading()) <= tolerance;
            if (isOnTarget)
            {
                stopTurn();
            }
        }
        return isOnTarget;
    }

    public void turnTask()
    {
        if (absoluteTurnTarget != null)
        {
            double currHeading = getHeading();
            double output = turnPidController.PIDControl(absoluteTurnTarget, currHeading);
            leftRearWheel.setPower(-output);
            rightRearWheel.setPower(output);
            telemetry.addData("TurnTask: CurrHeading", currHeading);
            telemetry.update();
        }
    }
}