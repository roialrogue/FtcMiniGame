package org.firstinspires.ftc.teamcode.NotInUse;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineBoard;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineCone;
import org.firstinspires.ftc.teamcode.VisonPiplines.MyGamePad;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



public class Auto1 {
    class Odometry {
        private static final boolean VERBOSE = false;
        public Object sensor;
        public double prevTimestamp;
        public double currTimestamp;
        public double prevPos;
        public double currPos;
        public double velocity;
        public double acceleration;

        public Odometry(
                Object sensor, double prevTimestamp, double currTimestamp, double prevPos, double currPos,
                double velocity, double acceleration) {
            this.sensor = sensor;
            this.prevTimestamp = prevTimestamp;
            this.currTimestamp = currTimestamp;
            this.prevPos = prevPos;
            this.currPos = currPos;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }   //Odometry
    }
}

