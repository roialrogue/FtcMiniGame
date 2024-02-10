package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Back 28", group = "CenterStage", preselectTeleOp = "Full")
public class Auto_BlueBackLimited extends CSBase {
    @Override
    public void runOpMode() {
        stageSide = side.b;
        color teamColor = color.b;
        setup(teamColor);

        // ---------------------
        // ------Main Code------
        // ---------------------

        pos = findPos();
//        int ID = setID(pos, teamColor);
        telemetry.addData("Team Prop X", x);
        telemetry.addData("Team Prop Position", pos);
        telemetry.update();
        purplePixel();
        drive(-2);
        turn(-90, dir.l);
        drive(tilesToInches(-1));
        turn(-90, dir.l);
        drive(tilesToInches(2));
        turn(-90, dir.l);
        drive(tilesToInches(0.5));
        ejectPixel();
        setSpeed(1000);
        drive(tilesToInches(0.2));


//        drive(tilesToInches(-2.1));
//        turn(90);
//        setSpeed(1000);
//        drive(tilesToInches(1.7));
//        setSpeed(2000);
//        ejectPixel();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        s(1);  // Pause to display final telemetry message.
    }
}