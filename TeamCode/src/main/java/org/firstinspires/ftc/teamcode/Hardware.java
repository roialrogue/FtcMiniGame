package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


public class Hardware {
    public DcMotor rightRearWheel;
    //CM0
    public DcMotor leftRearWheel;
    //CM3
    public DcMotor ArmMotor;
    //CM2
    public Servo LeftInTake;
    //"CS0"
    public Servo RightInTake;
    //"CS2"
    public BNO055IMU gyro;
    public RevColorSensorV3 color;
    public OpenCvCamera camera;
    private static Hardware myInstance = null;
    public static Hardware getInstance() {
        if (myInstance == null) {
            myInstance = new Hardware();
        }
        return myInstance;
    }

    public void init(HardwareMap hwMap) {

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


        ArmMotor = hwMap.get(DcMotor.class, "CM2");
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setDirection(DcMotor.Direction.FORWARD);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setPower(0);

        LeftInTake = hwMap.get(Servo.class, "CS0");

        RightInTake = hwMap.get(Servo.class, "CS2");

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
    }
    public void closeRight() {RightInTake.setPosition(.52);}
    public void closeLeft() {LeftInTake.setPosition(.65);}
    public void openRight() {RightInTake.setPosition(.18);}
    public void openLeft() {LeftInTake.setPosition(.95);}

//    public  void slidesTo(int slidePos, double power){
//        BeltMotor.setTargetPosition(slidePos);
//        BeltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BeltMotor.setPower(power);
//    }
//    public void slidesTo(int slidePos) {
//        slidesTo(slidePos,1.0);
//    }
}
