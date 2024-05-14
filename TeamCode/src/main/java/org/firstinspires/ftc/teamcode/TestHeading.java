package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDConstants.PIDConstantsHeading;


@Autonomous(name ="PID Heading SM")
public class TestHeading extends LinearOpMode {
    Hardware robot;
    ElapsedTime runtime = new ElapsedTime();
    private enum State
    {
        START_TURN,
        WAIT_FOR_TURN,
        DONE_WITH_TURN,
        START_DRIVE,
        WAIT_FOR_DRIVE,
        MOVE_ARM,
        WAIT_FOR_ARM,
        MOVE_CLAW,
        WAIT_FOR_CLAW,
        DONE
    };
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Hardware(hardwareMap, telemetry);
        robot.closeRight();
        robot.closeLeft();
        waitForStart();

        State state = State.MOVE_CLAW;
        while (opModeIsActive()) {
            switch (state)
            {
                case START_TURN:
                    // Start the turn.
                    robot.drivebase.turn(Math.toRadians(90),3);
                    state = State.WAIT_FOR_TURN;
                    runtime.reset();
                    break;
                case WAIT_FOR_TURN:
                    // Wait for the turn to finish.
                    if (robot.drivebase.turnOnTarget(Math.toRadians(2)))
                    {
                        state = State.DONE_WITH_TURN;
                    }
                    break;
                case DONE_WITH_TURN:
                    state = State.START_DRIVE;
                    break;
                case START_DRIVE:
                    robot.drivebase.drive(.2,24,3);
                    state = State.WAIT_FOR_DRIVE;
                    break;
                case WAIT_FOR_DRIVE:
                    if(robot.drivebase.driveOnTarget()) {
                        state = State.MOVE_ARM;
                    }
                    break;
                case MOVE_ARM:
                        robot.ArmMotor.setTargetPosition(-1000);
                        robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.ArmMotor.setPower(.7);
                        state = State.WAIT_FOR_ARM;
                        break;
                case WAIT_FOR_ARM:
                    if(!robot.ArmMotor.isBusy()) {
                        state = State.MOVE_CLAW;
                    }
                    break;
                case MOVE_CLAW:
                    robot.openLeft();
                    robot.openRight();
                    state = State.WAIT_FOR_CLAW;
                    break;
                case WAIT_FOR_CLAW:
                    if(runtime.seconds() > .5) {
                        state = State.DONE;
                    }
                    break;
                case DONE:
                default:
                    break;
            }
            robot.drivebase.turnTask();
            robot.drivebase.driveTask();
            telemetry.addData("State", state);
            telemetry.update();
        }
    }
}
