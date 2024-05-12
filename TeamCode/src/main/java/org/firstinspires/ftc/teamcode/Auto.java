package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();
    OpenCvCamera webCam;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private CameraPiplineBoard detector1;

    private CameraPiplineCone detector2;
    boolean editingConfig = true;
    boolean position1 = true;

    enum EditingMode {None, position1}

    public void runOpMode() {
        robot.init(hardwareMap);
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

    public  void turnDegrees(double speed, double degrees, double timeoutS, int turnDirection) {

        double inchesToTurn = (Math.PI * 14.75 * degrees)/360.0;

        if(opModeIsActive()) {
            int newTarget;

            newTarget = robot.leftRearWheel.getCurrentPosition() + (int)(inchesToTurn);

            if(turnDirection == 1) {
                robot.leftRearWheel.setTargetPosition(-newTarget);
                robot.rightRearWheel.setTargetPosition(newTarget);
            } else {
                robot.leftRearWheel.setTargetPosition(newTarget);
                robot.rightRearWheel.setTargetPosition(-newTarget);
            }

            robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftRearWheel.setPower(Math.abs(speed));
            robot.rightRearWheel.setPower(Math.abs(speed));

            while (opModeIsActive() && runtime.seconds() < timeoutS && robot.leftRearWheel.isBusy() || robot.rightRearWheel.isBusy()) { }

            robot.leftRearWheel.setPower(0);
            robot.rightRearWheel.setPower(0);
        }
    }

    public void turnDegree(double speed, double degrees, double tolerance, double timeout)
    {
        double targetHeading = robot.imu.getHeading() + degrees;

        robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeout &&
                Math.abs(targetHeading - imu.getHeading()) > tolerance)
        {
            double power = pidController.calculate(targetHeading, imu.getHeading());
            power = clip(power, -speed, speed);
            robot.leftRearWheel.setPower(power);
            robot.rightRearWheel.setPower(-power);
        }
        robot.leftRearWheel.setPower(0);
        robot.rightRearWheel.setPower(0);
    }
    double countsPerInch = 384.5 / (4 * Math.PI);
    public void driveInches(double speed, double distance, double timeoutS) {

        if(opModeIsActive()) {
            int newTarget;

            newTarget = robot.leftRearWheel.getCurrentPosition() + (int) (distance + countsPerInch);
            robot.leftRearWheel.setTargetPosition(newTarget);
            robot.rightRearWheel.setTargetPosition(newTarget);

            robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftRearWheel.setPower(Math.abs(speed));
            robot.rightRearWheel.setPower(Math.abs(speed));

            while (opModeIsActive() && runtime.seconds() < timeoutS && robot.leftRearWheel.isBusy() || robot.rightRearWheel.isBusy()) { }

            robot.leftRearWheel.setPower(0);
            robot.rightRearWheel.setPower(0);
        }
    }
}