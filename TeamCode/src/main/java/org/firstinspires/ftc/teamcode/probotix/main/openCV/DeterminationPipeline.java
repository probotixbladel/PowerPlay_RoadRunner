package org.firstinspires.ftc.teamcode.probotix.main.openCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class DeterminationPipeline extends OpenCvPipeline
{

    int treshold = 500;
    int minsize = 100;

    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);

    public enum MarkerPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    Mat mat2 = new Mat();
    int location = 0;

    private volatile MarkerPosition position = MarkerPosition.LEFT;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
        Core.inRange(YCrCb, scalarLowerYCrCb, scalarUpperYCrCb, YCrCb);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(YCrCb, contours, mat2, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

        for (MatOfPoint contour : contours) {
            Point[] contourArray = contour.toArray();

            MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
            Rect rect = Imgproc.boundingRect(areaPoints);

            if (rect.area() > minsize) {
                location = rect.x;
            }

            areaPoints.release();
            contour.release();

        }
    }

    @Override
    public void init(Mat input)
    {
        inputToCb(input);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        inputToCb(input);

        return input;
    }

    public int getLocation() {
        return location;
    }

    public MarkerPosition getAnalysis()
    {
        if (getLocation() > treshold){
            position = MarkerPosition.RIGHT;
        } else if(getLocation() <= treshold){
            position = MarkerPosition.CENTER;
        } else {
            position = MarkerPosition.LEFT;
        }

        return position;
    }
}