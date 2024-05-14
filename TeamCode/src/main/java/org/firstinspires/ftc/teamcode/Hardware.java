package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.NotInUse.Vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


public class Hardware {
    public DriveBase drivebase;

    public DcMotor ArmMotor;

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
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setPower(0);

        LeftInTake = hwMap.get(Servo.class, "CS0");

        RightInTake = hwMap.get(Servo.class, "CS1");

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