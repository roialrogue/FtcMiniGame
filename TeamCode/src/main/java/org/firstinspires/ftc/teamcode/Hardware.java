package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


public class Hardware {
    public DriveBase drivebase;

    public DcMotor ArmMotor;

    public Servo LeftInTake;

    public Servo RightInTake;

    public OpenCvCamera camera;

    public Hardware(HardwareMap hwMap) {
        drivebase = new DriveBase(hwMap);

        ArmMotor = hwMap.get(DcMotor.class, "CM2");
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setDirection(DcMotor.Direction.REVERSE);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setPower(0);

        LeftInTake = hwMap.get(Servo.class, "CS0");

        RightInTake = hwMap.get(Servo.class, "CS1");

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

    }

    public void closeRight() {
        RightInTake.setPosition(.52);
    }

    public void closeLeft() {
        LeftInTake.setPosition(.65);
    }

    public void openRight() {
        RightInTake.setPosition(.30);
    }
    public void openLeft() {
        LeftInTake.setPosition(.86);
    }
}