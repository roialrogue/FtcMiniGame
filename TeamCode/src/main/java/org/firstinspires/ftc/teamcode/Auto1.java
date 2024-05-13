package org.firstinspires.ftc.teamcode;
import android.app.DownloadManager;

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
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "Auto1")
public class Auto1 extends LinearOpMode {
    Hardware robot;

    OpenCvCamera webCam;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime runtime = new ElapsedTime();
    private CameraPiplineBoard detector1;
    private CameraPiplineCone detector2;
    boolean editingConfig = true;
    boolean position1 = true;
    enum EditingMode {None, position1}

    private DriveBase turnDriveBase;

    public void runOpMode() throws InterruptedException {
        robot = new Hardware(hardwareMap);
        myGamePad myGamepad = new myGamePad(gamepad1);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        CameraPiplineBoard detector1 = new CameraPiplineBoard(telemetry);
        CameraPiplineCone detector2 = new CameraPiplineCone(telemetry);
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webCam.openCameraDevice();
        webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard.getInstance().startCameraStream(webCam, 0);

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

        turnDriveBase.drive(.2,50,5);
        robot.ArmMotor.setTargetPosition(1000);
        robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmMotor.setPower(.7);

//        if (position1) {
//            //left position april 1-3
//            if(CameraPiplineBoard.blue) {
//                //Board is blue
//
//            } else {
//                //Board is red
//
//            }
//        } else {
//            //right position april 3-6
//            if(CameraPiplineBoard.blue) {
//                //Board is blue
//
//            } else {
//                //Board is red
//
//            }
//        }
//
//        webCam.setPipeline(detector2);
//        FtcDashboard.getInstance().startCameraStream(webCam, 0);
//        webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//
//        if(CameraPiplineCone.blue){
//            //cone is blue
//
//        } else {
//            //cone is red
//
//        }
//
//        //add park

    }
}
