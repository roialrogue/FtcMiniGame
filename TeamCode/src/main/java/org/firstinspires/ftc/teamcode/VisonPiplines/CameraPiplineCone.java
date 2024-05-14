package org.firstinspires.ftc.teamcode.VisonPiplines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class CameraPiplineCone extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();

    private Mat workingMatrix1 = new Mat();

    private Mat workingMatrix2 = new Mat();
    public static boolean red;
    public static boolean blue;
    public static int base;
    public static int matArowStart = 0;
    public static int matArowEnd = 720;
    public static int matAcolStart = 0;
    public static int matAcolEnd = 1280;
    Telemetry telemetry;
    public CameraPiplineCone(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public final Mat processFrame(Mat input) {

        if (input.empty()) {
            return input;
        }

        Imgproc.cvtColor(input, workingMatrix, Imgproc.COLOR_RGB2HSV);

        Scalar lowVal, highVal;
        lowVal = new Scalar(170, 50, 50);
        highVal = new Scalar(179, 255, 255);
        Core.inRange(workingMatrix, lowVal, highVal, workingMatrix1);

        lowVal = new Scalar(0, 50, 50);
        highVal = new Scalar(10, 255, 255);
        Core.inRange(workingMatrix, lowVal, highVal, workingMatrix2);

        Core.add(workingMatrix1, workingMatrix2, workingMatrix);


        Mat all = workingMatrix.submat(matArowStart,matArowEnd, matAcolStart, matAcolEnd);

        Imgproc.rectangle(workingMatrix, new Rect(matAcolStart, matArowStart, (matAcolEnd - matAcolStart), (matArowEnd - matArowStart)), new Scalar(0, 255, 0));

        double Value = Core.sumElems(all).val[0] / (all.rows() * all.cols());

        red = false;
        blue = false;
        base = 10;

        if (Value > base) {
            telemetry.addData("Found color","red");
            telemetry.update();
            red = true;
        } else {
            telemetry.addData("Found color", "blue");
            telemetry.update();
            blue = true;
        }
        telemetry.update();
        return workingMatrix;
    }
}
