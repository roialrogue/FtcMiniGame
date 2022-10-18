package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Terminator Mode")
public class Drive extends OpMode {
    Warbotron warbotron;

    @Override
    public void init() {
        warbotron = new Warbotron(hardwareMap);
    }

    @Override
    public void loop() {
        double power = 1;

        if(gamepad1.right_bumper) power = 0.5;

        warbotron.frontLeft.setPower(-gamepad1.left_stick_y * power);
        warbotron.frontRight.setPower(-gamepad1.right_stick_y * power);
        warbotron.backLeft.setPower(-gamepad1.left_stick_y * power);
        warbotron.backRight.setPower(-gamepad1.right_stick_y * power);
        warbotron.hammer1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        warbotron.hammer2.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
    }
}
