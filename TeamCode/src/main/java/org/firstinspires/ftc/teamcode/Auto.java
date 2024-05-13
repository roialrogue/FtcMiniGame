package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineBoard;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineCone;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    OpenCvCamera webCam;
    public DcMotor ArmMotor;

    public Servo LeftInTake;

    public Servo RightInTake;

    public BHI260IMU imu;

    public OpenCvCamera camera;
    private DriveBase turnDriveBase;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private CameraPiplineBoard detector1;
    private CameraPiplineCone detector2;
    boolean editingConfig = true;
    boolean position1 = true;

    enum EditingMode {None, position1}

    public Auto(HardwareMap hwMap)
    {
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

        imu = hwMap.get(BHI260IMU.class,"imu");
        IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();
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

    public void runOpMode() {
        myGamePad myGamepad = new myGamePad(gamepad1);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        CameraPiplineBoard detector1 = new CameraPiplineBoard(telemetry);
        CameraPiplineCone detector2 = new CameraPiplineCone(telemetry);
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webCam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

        webCam.setPipeline(detector1);

        EditingMode editingMode = EditingMode.None;

        while (editingConfig) {
            if (myGamepad.isXPressed()) {
                switch (editingMode) {
                    case None:
                        editingMode = EditingMode.position1;
                        break;
                    case position1:
                        editingMode = EditingMode.None;
                        break;
                }
            }

            switch (editingMode) {
                case None:
                    break;
                case position1:
                    if (myGamepad.isRightBumperPressed()) {
                        position1 = true;
                    } else if (myGamepad.isLeftBumperPressed()) {
                        position1 = false;
                    }
                    break;
            }
            telemetry.addData("Starting on side 1 (April 1-3)", position1);
            telemetry.update();

            if (myGamepad.isleftstickbuttonPressed()) {
                editingConfig = false;
            }
        }

        telemetry.addData("Ready", "Editing auto is done");
        telemetry.update();

        waitForStart();
        webCam.stopStreaming();


        if (position1) {
            //left position april 1-3
            if(CameraPiplineBoard.blue) {
                //Board is blue

            } else {
                //Board is red

            }
        } else {
            //right position april 3-6
            if(CameraPiplineBoard.blue) {
                //Board is blue

            } else {
                //Board is red

            }
        }

        webCam.setPipeline(detector2);
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

        if(CameraPiplineCone.blue){
            //cone is blue

        } else {
            //cone is red

        }

        //add park

    }
}