package org.firstinspires.ftc.teamcode.NotInUse;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineBoard;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineCone;
import org.firstinspires.ftc.teamcode.VisonPiplines.myGamePad;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


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

    public void runOpMode() throws InterruptedException {
        robot = new Hardware(hardwareMap, telemetry);
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
        robot.closeLeft();
        robot.closeRight();
        robot.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        webCam.stopStreaming();

        robot.ArmMotor.setTargetPosition(-1000);
        robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmMotor.setPower(.7);
        while(robot.ArmMotor.isBusy()) {}


//        if (position1) {
//            //left position april 1-3
//            if(CameraPiplineBoard.blue) {
//                //Board is blue
//                driveInches(.2,50,5);
//
//            } else {
//                //Board is red
//                driveInches(.2,50,5);
//            }
//            robot.ArmMotor.setTargetPosition(1000);
//            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.ArmMotor.setPower(.7);
//            robot.drivebase.turn(90,3);
//            robot.drivebase.turnTask();
//            while(!robot.drivebase.turnOnTarget(2));
//            robot.drivebase.stopTurn();
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

    private static final double countsPerInch = 384.5 / (4 * Math.PI);

    public void driveInches(double speed, double distance, double timeouts) {
        int newTarget;

        newTarget = robot.leftRearWheel.getCurrentPosition() + (int) (distance * countsPerInch);
        robot.leftRearWheel.setTargetPosition(newTarget);
        robot.rightRearWheel.setTargetPosition(newTarget);

        robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.leftRearWheel.setPower(Math.abs(speed));
        robot.rightRearWheel.setPower(Math.abs(speed));

        while (opModeIsActive() && runtime.seconds() < timeouts && robot.leftRearWheel.isBusy() || robot.rightRearWheel.isBusy()) { }

        robot.leftRearWheel.setPower(0);
        robot.rightRearWheel.setPower(0);
    }
}
