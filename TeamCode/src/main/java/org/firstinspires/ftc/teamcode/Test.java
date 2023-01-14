package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo armRight = hardwareMap.servo.get("armRight");
        Servo armLeft = hardwareMap.servo.get("armLeft");

        Servo grabber = hardwareMap.servo.get("grabber");
        DistanceSensor s = hardwareMap.get(DistanceSensor.class, "grabberSensor");

        armRight.setDirection(Servo.Direction.REVERSE);

        boolean i = true;

        waitForStart();

        while (opModeIsActive()){
            armRight.setPosition(i ? 0.36:0.56);
            armLeft.setPosition(i ? 0.36:0.56);

            if (A_pressed()){
                i = !i;
            }

            if (s.getDistance(DistanceUnit.CM) < 8.5) grabber.setPosition(0.775);
            else grabber.setPosition(0.45);

            telemetry.addData("s", s.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

    private boolean a = true;
    public boolean A_pressed(){
        if (gamepad1.a){
            if(a){
                a = false;
                return true;
            }
        } else a = true;
        return false;
    }
}