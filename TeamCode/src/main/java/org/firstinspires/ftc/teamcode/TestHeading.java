package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PIDConstants.PIDConstantsHeading;


@Autonomous(name ="PID Heading SM")
public class TestHeading extends LinearOpMode {
    private enum State
    {
        START_TURN,
        WAIT_FOR_TURN,
        DONE_WITH_TURN,
        DONE
    };
    private DriveBase turnDriveBase;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        State state = State.START_TURN;
        while (opModeIsActive()) {
            switch (state)
            {
                case START_TURN:
                    // Start the turn.
                    turnDriveBase.turn(Math.toRadians(PIDConstantsHeading.referenceAngle));
                    state = State.WAIT_FOR_TURN;
                    break;
                case WAIT_FOR_TURN:
                    // Wait for the turn to finish.
                    if (turnDriveBase.turnOnTarget(Math.toRadians(2.0)))
                    {
                        state = State.DONE_WITH_TURN;
                    }
                    break;
                case DONE_WITH_TURN:
                    telemetry.addData("PIDHeadingStatus", "Done");
                    state = State.DONE;
                    break;
                case DONE:
                default:
                    break;
            }
            turnDriveBase.turnTask();
            telemetry.addData("State", state);
            telemetry.update();
        }
    }
}
