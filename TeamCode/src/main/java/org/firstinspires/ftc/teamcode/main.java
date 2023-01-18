package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class main extends LinearOpMode {


    movement movement = new movement();

    @Override
    public void runOpMode() {
        movement.Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        movement.Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        movement.Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
        movement.Motor4 = hardwareMap.get(DcMotor.class, "Motor4");

        while (opModeIsActive()){
            gamepader();
        }
    }
    public void gamepader(){
        if (Math.abs(gamepad1.right_stick_y) > Math.abs(gamepad1.right_stick_x)) {
            movement.verticalMovement(gamepad1.right_stick_y);
        } else if (Math.abs(gamepad1.right_stick_y) < Math.abs(gamepad1.right_stick_x)) {
            movement.horizontalMovement(gamepad1.right_stick_x);
        } else {
            movement.stop();
        }
    }
}