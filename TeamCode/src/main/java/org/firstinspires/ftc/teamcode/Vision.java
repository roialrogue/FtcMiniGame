package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineBoard;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineCone;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Vision
{
    private CameraPiplineBoard detector1;
    private CameraPiplineCone detector2;

    public Vision(HardwareMap hardwareMap)
    {
//        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//        WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");
//        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        detector1 = new CameraPiplineBoard(telemetry);
//        detector2 = new CameraPiplineCone(telemetry);
//        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webCam.openCameraDevice();
//        webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//        webCam.setPipeline(detector1);
//    }
}
