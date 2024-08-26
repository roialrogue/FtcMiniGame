package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {
    Hardware robot;



    @Override
    public void runOpMode() {
        robot = new Hardware(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.closeLeft();
        robot.closeRight();
        double slowMode = .6;
        waitForStart();

        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower  = Range.clip(drive + turn, -1, 1) * slowMode ;
            rightPower = Range.clip(drive - turn, -1, 1) * slowMode;

            robot.drivebase.setDrivePower(leftPower,rightPower);

            if(gamepad1.right_bumper) {
                robot.openRight();
            } else {
                robot.closeRight();
            }

            if(gamepad1.left_bumper) {
                robot.openLeft();
            } else {
                robot.closeLeft();
            }

            if(gamepad1.x) {
                robot.openLeft();
                robot.openRight();
            }

            if(gamepad2.right_stick_y > .1) {
                robot.ArmMotor.setPower(.7);
            } else if(gamepad2.right_stick_y < -.1) {
                robot.ArmMotor.setPower(-.7);
            } else {
                robot.ArmMotor.setPower(0);
            }

            telemetry.addData("Heading", robot.drivebase.getHeading());
        }
    }
}
