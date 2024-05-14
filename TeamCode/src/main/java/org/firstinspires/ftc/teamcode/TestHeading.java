package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        DONE
    };
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Hardware(hardwareMap, telemetry);

        waitForStart();

        State state = State.START_TURN;
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
                    if (robot.drivebase.turnOnTarget(2))
                    {
                        state = State.DONE_WITH_TURN;
                    }
                    break;
                case DONE_WITH_TURN:
                    state = State.START_DRIVE;
                    break;
                case START_DRIVE:
                    robot.drivebase.drive(.2,50,3);
                    state = State.WAIT_FOR_DRIVE;
                    break;
                case WAIT_FOR_DRIVE:
                    if(robot.drivebase.driveOnTarget()) {
                        state = State.DONE;
                    }
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
