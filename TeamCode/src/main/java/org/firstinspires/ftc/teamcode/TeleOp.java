package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.closeLeft();
        robot.closeRight();
        robot.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -.6, .6) ;
            rightPower   = Range.clip(drive - turn, -.6, .6) ;

            robot.leftRearWheel.setPower(leftPower);
            robot.rightRearWheel.setPower(rightPower);

            if(gamepad2.right_bumper) {
                robot.openRight();
            } else {
                robot.closeRight();
            }

            if(gamepad2.left_bumper) {
                robot.openLeft();
            } else {
                robot.closeLeft();
            }

            if(gamepad2.x) {
                robot.openLeft();
                robot.openRight();
            } else  {
                robot.closeLeft();
                robot.closeRight();
            }

            if(gamepad2.right_stick_y > .1) {
                robot.ArmMotor.setPower(1);
            } else if(gamepad2.right_stick_y < -.1) {
                robot.ArmMotor.setPower(-1);
            }

            //a run to position code for arm
        }
    }
}
