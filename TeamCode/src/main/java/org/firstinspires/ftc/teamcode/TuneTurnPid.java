package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PIDConstants.PIDConstantsHeading;

@Autonomous(name ="TuneTurnPid")
public class TuneTurnPid extends LinearOpMode
{
    Hardware robot;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    double prevTarget;

    private enum State
    {
        SET_TARGET,
        WAIT_FOR_COMPLETION
    }

    public void runOpMode() throws InterruptedException
    {
        robot = new Hardware(hardwareMap,telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        prevTarget = PIDConstantsHeading.referenceAngle;

        waitForStart();

        State state= State.SET_TARGET;
        while (opModeIsActive())
        {
            double currTarget = PIDConstantsHeading.referenceAngle;
            switch (state)
            {
                case SET_TARGET:
                    if (currTarget != prevTarget)
                    {
                        robot.drivebase.turn(currTarget, 3.0);
                        prevTarget = currTarget;
                        state = State.WAIT_FOR_COMPLETION;
                    }
                    break;
                case WAIT_FOR_COMPLETION:
                    if (robot.drivebase.turnOnTarget(2.0))
                    {
                        state = State.SET_TARGET;
                    }
                    break;
            }
        }
    }
}
