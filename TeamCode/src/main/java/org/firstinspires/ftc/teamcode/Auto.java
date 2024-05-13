package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineBoard;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineCone;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    Hardware robot;

    OpenCvCamera webCam;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime runtime = new ElapsedTime();
    private CameraPiplineBoard detector1;
    private CameraPiplineCone detector2;
    boolean editingConfig = true;
    boolean position1 = true;

    enum EditingMode {None, position1}

    private enum State {
        Drive_To_Basket,
        TURN_At_Basket,
        Square_To_Cone,
        Move_Cone_To_Mark,
        Done
    }

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

        State state = State.Drive_To_Basket;
        while (opModeIsActive() & !isStopRequested()) {
            switch (state) {
                case Drive_To_Basket:
                    if (CameraPiplineBoard.red) {
                        robot.drivebase.drive(.2, 50, 5);
                    } else if (CameraPiplineBoard.blue) {
                        robot.drivebase.drive(.2, 65, 7);
                    }
                    robot.ArmMotor.setTargetPosition(1000);
                    robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.ArmMotor.setPower(.7);
                    state = State.TURN_At_Basket;
                    break;
                case TURN_At_Basket:
                    if (robot.drivebase.driveOnTarget()) {
                        robot.drivebase.turn(Math.toRadians(90));
                        state = State.Square_To_Cone;
                    }
                    break;
                case Square_To_Cone:
                    if (robot.drivebase.turnOnTarget(Math.toRadians(2.0))) {
                        robot.drivebase.turn(-90);
                    }
                    state = State.Move_Cone_To_Mark;
                    break;

                case Move_Cone_To_Mark:

                    state = State.Done;
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