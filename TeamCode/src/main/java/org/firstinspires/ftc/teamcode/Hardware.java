package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


public class Hardware {
    public DriveBase drivebase;

    public DcMotor ArmMotor;

    public DcMotor leftRearWheel;
    public DcMotor rightRearWheel;

    public Servo LeftInTake;

    public Servo RightInTake;

    public Vision vision;

    public OpenCvCamera camera;

    public Hardware(HardwareMap hwMap, Telemetry telemetry) {
        drivebase = new DriveBase(hwMap, telemetry);

        ArmMotor = hwMap.get(DcMotor.class, "CM2");
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setDirection(DcMotor.Direction.REVERSE);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setPower(0);

        LeftInTake = hwMap.get(Servo.class, "CS0");

        RightInTake = hwMap.get(Servo.class, "CS1");

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

//        vision = new Vision(hwMap);
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
    }

    public void closeRight() {
        RightInTake.setPosition(.55);
    }

    public void closeLeft() {
        LeftInTake.setPosition(.60);
    }

    public void openRight() {
        RightInTake.setPosition(.30);
    }
    public void openLeft() {
        LeftInTake.setPosition(.86);
    }
}