package org.firstinspires.ftc.teamcode.NotInUse;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineBoard;
import org.firstinspires.ftc.teamcode.VisonPiplines.CameraPiplineCone;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision {
    private CameraPiplineBoard detector1;
    private CameraPiplineCone detector2;
    public OpenCvCamera camera;
    public Vision(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        detector1 = new CameraPiplineBoard(telemetry);
        detector2 = new CameraPiplineCone(telemetry);
        camera.openCameraDevice();
        camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
        camera.setPipeline(detector1);
//    }
    }

    public void cvVision(OpenCvPipeline detector, int pramWidth, int pramHeight, OpenCvCameraRotation CameraRotation)
    {
        camera.startStreaming(pramWidth, pramHeight, CameraRotation);
        camera.setPipeline(detector);
    }

}
