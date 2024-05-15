package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineBoard;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineCone;
import org.firstinspires.ftc.teamcode.VisonPiplines.MyGamePad;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    Hardware robot;

    OpenCvCamera webCam;
    ElapsedTime runtime = new ElapsedTime();
    boolean editingConfig = true;
    boolean position1 = true;
    enum EditingMode {None, position1}

    public CameraPiplineCone detector2;
    public CameraPiplineBoard detector1;


    private enum State {
        Drive_To_Basket,
        TURN_At_Basket,
        Drop_Pixel,
        Square_To_Cone,
        Arm_Down,
        Grab_Cone,
        Grab_cone_claw,
        Move_Cone_To_Mark,
        Move_Cone_On_Mark,
        OPEN_CLAW,
        BACK_UP,
        Done
    }

    public void runOpMode() throws InterruptedException {
        robot = new Hardware(hardwareMap, telemetry);
        MyGamePad myGamepad = new MyGamePad(gamepad1);
        detector1 = new CameraPiplineBoard(telemetry);
        detector2 = new CameraPiplineCone(telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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

        waitForStart();
        webCam.stopStreaming();

        State state = State.Drive_To_Basket;
        while (opModeIsActive() & !isStopRequested()) {
            switch (state) {
                case Drive_To_Basket:
                    if (CameraPiplineBoard.red) {
                        robot.drivebase.drive(.2, 50, 5);
                    } else {
                        //Blue
                        robot.drivebase.drive(.2, 65, 7);
                    }
                    robot.ArmMotor.setTargetPosition(1000);
                    robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.ArmMotor.setPower(.7);
                    state = State.TURN_At_Basket;
                    break;
                case TURN_At_Basket:
                    if (robot.drivebase.driveOnTarget()) {
                        robot.drivebase.absoluteTurn(90,3);
                        state = State.Drop_Pixel;
                    }
                    break;

                case Drop_Pixel:
                    if (robot.drivebase.turnOnTarget(2.0)) {
                        robot.openLeft();
                        robot.openRight();
                        state = State.Square_To_Cone;
                        runtime.reset();
                    }
                    break;
                case Square_To_Cone:
                    if (runtime.seconds() > 0.5) {
                        robot.drivebase.absoluteTurn(-90,3);
                        state = State.Arm_Down;
                    }
                    break;
                case Arm_Down:
                    if (robot.drivebase.turnOnTarget(2.0)) {
                        robot.ArmMotor.setTargetPosition(200);
                        robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.ArmMotor.setPower(.7);
                        webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                        webCam.setPipeline(detector2);
                        state = State.Grab_Cone;
                    }
                    break;
                case Grab_Cone:
                    if(!robot.ArmMotor.isBusy()) {
                        robot.drivebase.drive(.2,12,3);
                        state = State.Grab_cone_claw;
                    }
                    break;
                case Grab_cone_claw:
                    if(robot.drivebase.driveOnTarget()) {
                        robot.closeLeft();
                        robot.closeRight();
                        state = State.Move_Cone_To_Mark;
                        runtime.reset();
                    }
                    break;
                case Move_Cone_To_Mark:
                    if(runtime.seconds() > 0.5) {
                        robot.drivebase.drive(.2,54,5);
                        state = State.Move_Cone_On_Mark;
                    }
                    break;
                case Move_Cone_On_Mark:
                    if(robot.drivebase.driveOnTarget()) {
                        if (CameraPiplineCone.red) {
                            robot.drivebase.absoluteTurn(-30,1);
                        } else {
                            //Blue
                            robot.drivebase.absoluteTurn(30,1);
                        }
                        state = State.Done;
                    }
                    break;
                case Done:
                default:
                    break;
            }
            robot.drivebase.turnTask();
            robot.drivebase.driveTask();
        }
    }
}