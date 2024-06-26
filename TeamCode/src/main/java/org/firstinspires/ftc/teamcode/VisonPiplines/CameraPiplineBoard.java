package org.firstinspires.ftc.teamcode.VisonPiplines;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class CameraPiplineBoard extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();

    private Mat workingMatrix1 = new Mat();

    private Mat workingMatrix2 = new Mat();
    public static boolean red;
    public static boolean blue;
    public static int matArowStart = 0;
    public static int matArowEnd = 360;
    public static int matAcolStart = 0;
    public static int matAcolEnd = 1280;
    public static int matBrowStart = 360;
    public static int matBrowEnd = 720;
    public static int matBcolStart = 0;
    public static int matBcolEnd = 1280;
    Telemetry telemetry;

    public CameraPiplineBoard(Telemetry telemetry) {
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

        Mat low = workingMatrix.submat(matArowStart,matArowEnd, matAcolStart, matAcolEnd);
        Mat high = workingMatrix.submat(matBrowStart, matBrowEnd, matBcolStart, matBcolEnd);

        Imgproc.rectangle(workingMatrix, new Rect(matAcolStart, matArowStart, (matAcolEnd - matAcolStart), (matArowEnd - matArowStart)), new Scalar(60, 0, 0));
        Imgproc.rectangle(workingMatrix, new Rect(matBcolStart,matBrowStart , (matBcolEnd - matBcolStart), (matBrowEnd - matBrowStart)), new Scalar(5, 0, 0));

        double lowValue = Core.sumElems(low).val[0] / (low.rows() * low.cols());
        double highValue = Core.sumElems(high).val[0] / (high.rows() * high.cols());

        red = false;
        blue = false;

        if (lowValue > highValue) {
            telemetry.addData("Found color","Blue");
            red = true;
        } else {
            telemetry.addData("Found color","Red");
            blue = true;
        }
        telemetry.update();
        return workingMatrix;
    }
}
