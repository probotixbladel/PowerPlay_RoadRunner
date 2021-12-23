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

/*
     - It is strongly recommended to read the user docs on pipelines: github.com/OpenFTC/EasyOpenCV/blob/master/doc/user_docs/pipelines_overview.md
     
     This pipeline:
     - converts the original camera image to an YCrCb image =>
        Y is the luma component
        Cr red difference chroma component
        Cb blue difference chroma component
        For more info see Wikipedia
     - then it uses the Cb component for further contour detection (extract channel)
     - the Cb component is filtered: if within filter limits =>
            pixels are set to 255 (white) if not pixels are set to 0 (black);
            black-white is necessary for contour detection
     - In OpenCV, finding contours is like finding white object from black background.
            So remember, object to be found should be white and background should be black
            
     
     
     


*/

public class DeterminationPipeline extends OpenCvPipeline
{

    int treshold = 500; // location threshold
    int minsize = 100; // minimum size of bounding rectangle

    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0.0, 0.0); // filter limit 1
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0); // filter limit 2

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
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);        // conversion to YCrCb
        Core.extractChannel(YCrCb, Cb, 2);                              // extract Cb component (channel 0: Y; channel 1: Cr; channel2: Cb)
        Core.inRange(YCrCb, scalarLowerYCrCb, scalarUpperYCrCb, YCrCb); // checks if array elements lie between the elements of two other arrays; output to YCrCb Mat

        List<MatOfPoint> contours = new ArrayList<>(); // list of contours 
        Imgproc.findContours(YCrCb, contours, mat2, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE); // find contours see opencv docs

        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0)); // draw contours on the input image
        
        // processing of the contours and their locations
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
    
    
    // determine the postion of the marker (duck or team shipping element)
    //tresholod = 500
    public MarkerPosition getAnalysis()
    {
        if (getLocation() < treshold){
            position = MarkerPosition.LEFT;
        } else if(getLocation() >= treshold){
            position = MarkerPosition.CENTER;
        } else if(getLocation() < 50){
            position = MarkerPosition.RIGHT;  // ?? only two locations visible? Default position is LEFT.
        } else if(getLocation() > 1000) {
            position = MarkerPosition.RIGHT;
        } else {
            position = MarkerPosition.RIGHT;
        }

        return position;
    }
}
