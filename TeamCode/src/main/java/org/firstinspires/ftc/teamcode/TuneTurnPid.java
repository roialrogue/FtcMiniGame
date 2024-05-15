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
    Double prevTarget = null;

    private enum State
    {
        SET_TARGET,
        WAIT_FOR_COMPLETION
    }

    public void runOpMode() throws InterruptedException
    {
        robot = new Hardware(hardwareMap,telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        State state = State.SET_TARGET;
        while (opModeIsActive())
        {
            double currTarget = PIDConstantsHeading.referenceAngle;
            switch (state)
            {
                case SET_TARGET:
                    if (prevTarget == null || currTarget != prevTarget)
                    {
                        robot.drivebase.turn(currTarget, 3.0);
                        telemetry.addData("TuneTurnPID.settingTarget", currTarget);
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
            robot.drivebase.turnTask();
            telemetry.addData("TuneTurnPID.state", state);
            telemetry.addData("TuneTurnPID.currTarget", currTarget);
            telemetry.addData("TuneTurnPID.currHeading", robot.drivebase.getHeading());
            telemetry.update();
        }
    }
}