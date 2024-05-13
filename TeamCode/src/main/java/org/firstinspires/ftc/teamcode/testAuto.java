package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineBoard;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineCone;
import org.openftc.easyopencv.OpenCvCamera;


@Autonomous(name = "TestAuto")
public class testAuto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.leftRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        driveInches(.2,24,5);
    }

    double countsPerInch = 384.5 / (4 * Math.PI);

    public void driveInches(double speed, double distance, double timeoutS) {

        if (opModeIsActive()) {
            int newTarget;

            newTarget = robot.leftRearWheel.getCurrentPosition() + (int) (distance * countsPerInch);
            robot.leftRearWheel.setTargetPosition(newTarget);
            robot.rightRearWheel.setTargetPosition(newTarget);

            robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftRearWheel.setPower(Math.abs(speed));
            robot.rightRearWheel.setPower(Math.abs(speed));

            while (opModeIsActive() && runtime.seconds() < timeoutS && robot.leftRearWheel.isBusy() || robot.rightRearWheel.isBusy()) {
            }

            robot.leftRearWheel.setPower(0);
            robot.rightRearWheel.setPower(0);
        }
    }
}