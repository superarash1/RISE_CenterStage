package org.firstinspires.ftc.robotcontroller.Hardware.Sensors.Camera.OpenCV.VisionPipelines;

import static org.opencv.core.Core.inRange;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PowerPlayPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    static final Scalar GREEN = new Scalar(0, 255, 0);

    public boolean RED = true;
    public boolean BLUE = true;
    public boolean YELLOW = true;

    public int redContourCount = 0;
    public int blueContourCount = 0;
    public int yellowContourCount = 0;

    public List<Rect> redRect;
    public List<Rect> blueRect;
    public List<Rect> yellowRect;

    public Rect RedRect;
    public Rect BlueRect;
    public Rect YellowRect;

    public List<MatOfPoint> redContours;
    public List<MatOfPoint> blueContours;
    public List<MatOfPoint> yellowContours;

    public MatOfPoint biggestRedContour;
    public MatOfPoint biggestBlueContour;
    public MatOfPoint biggestYellowContour;

    public PowerPlayPipeline(Telemetry telemetry) {
        redContours = new ArrayList<MatOfPoint>();
        redRect = new ArrayList<Rect>();
        RedRect = new Rect();
        biggestRedContour = new MatOfPoint();

        blueContours = new ArrayList<MatOfPoint>();
        blueRect = new ArrayList<Rect>();
        BlueRect = new Rect();
        biggestBlueContour = new MatOfPoint();

        yellowContours = new ArrayList<MatOfPoint>();
        yellowRect = new ArrayList<Rect>();
        YellowRect = new Rect();
        biggestYellowContour = new MatOfPoint();

        this.telemetry = telemetry;
    }

    // Filters the contours to be greater than a specific area in order to be tracked
    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > 25;
    }

    // Red masking thresholding values:
    Scalar lowRed = new Scalar(0, 165, 0); //10, 100, 50
    Scalar highRed = new Scalar(255, 255, 255); //35, 255, 255

    // Blue masking thresholding values:
    Scalar lowBlue = new Scalar(0, 0, 140); //10, 100, 50
    Scalar highBlue = new Scalar(255, 255, 255); //35, 255, 255

    // Yellow masking thresholding values:
    Scalar lowYellow = new Scalar(10, 50, 100); //10, 100, 50
    Scalar highYellow = new Scalar(50, 255, 255); //35, 255, 255

    // Mat object for the red and blue mask
    Mat maskRed = new Mat();
    Mat maskBlue = new Mat();
    Mat maskYellow = new Mat();

    // Mat object for YCrCb color space
    Mat YCrCb = new Mat();

    Mat HSV = new Mat();

    // Kernel size for blurring
    Size kSize = new Size(5, 5);
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2 * 2) + 1, (2 * 2) + 1));

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(YCrCb, YCrCb, kernel);
        Imgproc.GaussianBlur(YCrCb, YCrCb, kSize, 0);

        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Imgproc.erode(HSV, HSV, kernel);
        Imgproc.GaussianBlur(HSV, HSV, kSize, 0);

        if (RED) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskRed"
            inRange(YCrCb, lowRed, highRed, maskRed);

            // Clears the arraylists
            redContours.clear();
            redRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, redContours, -1, GREEN); //input

            // Iterates through each contour
            for (int i = 0; i < redContours.size(); i++) {

//                Imgproc.HoughCircles(redContours.get(i), circles, Imgproc.HOUGH_GRADIENT, Math.PI/180, 50, 1, 50);
                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(redContours.get(i))) {

                    // TODO: move outside of for loop to optimize potentially
                    biggestRedContour = Collections.max(redContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contourand the draws it
                    RedRect = Imgproc.boundingRect(biggestRedContour);

                    Imgproc.rectangle(input, RedRect, GREEN, 2);
                }
            }
            maskRed.release();
        }

        if (BLUE) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskBlue"
            inRange(YCrCb, lowBlue, highBlue, maskBlue);

            // Clears the arraylists
            blueContours.clear();
            blueRect.clear();

            //TODO: Canny edge detection?

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, blueContours, -1, GREEN); //input

            // Iterates through each contour
            for (int i = 0; i < blueContours.size(); i++) {
                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(blueContours.get(i))) {
                    biggestBlueContour = Collections.max(blueContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contourand the draws it
                    BlueRect = Imgproc.boundingRect(biggestBlueContour);
                    Imgproc.rectangle(input, BlueRect, GREEN, 2);
                }
            }
            maskBlue.release();
        }

        if (YELLOW) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskYellow"
            inRange(HSV, lowYellow, highYellow, maskYellow);

            // Clears the arraylists
            yellowContours.clear();
            yellowRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskYellow, yellowContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, yellowContours, -1, GREEN); //input

            // Iterates through each contour
            for (int i = 0; i < yellowContours.size(); i++) {
                // Filters out contours with an area less than 50 (defined in the filter contours method)
//                if (filterContours(yellowContours.get(i))) {
                    // Creates a bounding rect around each contourand the draws it

                    biggestYellowContour = Collections.max(yellowContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    YellowRect = Imgproc.boundingRect(biggestYellowContour);
//                    yellowRect.add(Imgproc.boundingRect(yellowContours.get(i)));
                    Imgproc.rectangle(input, YellowRect, GREEN, 2);
                    // Creates a count for the amount of yellow contours on the the screen
                    yellowContourCount++;
//                }
            }
            maskYellow.release();
        }
        redContourCount = 0;
        blueContourCount = 0;
        yellowContourCount = 0;

        YCrCb.release();

        //TODO: move this when actually using code (this is just for EasyOpenCV sim)

        return input;
    }

    public double getRectY(Rect rect){
        return rect.y + (rect.height/2);
    }

    public double getRectX(Rect rect){
        return rect.x + (rect.width/2);
    }

    public void Telemetry(){
        if (!YellowRect.empty()){
            telemetry.addData("Junction X Pos", getRectX(YellowRect));
            telemetry.addData("Junction Width", YellowRect.width);
        }

        if (!RedRect.empty()){
            telemetry.addData("Conestack X Pos", getRectX(RedRect));
            telemetry.addData("Conestack Width", RedRect.width);
        }

        if (!BlueRect.empty()){
            telemetry.addData("Conestack X Pos", getRectX(BlueRect));
            telemetry.addData("Conestack Width", BlueRect.width);
        }
    }
}