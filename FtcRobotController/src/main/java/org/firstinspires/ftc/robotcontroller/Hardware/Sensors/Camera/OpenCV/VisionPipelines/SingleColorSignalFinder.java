package org.firstinspires.ftc.robotcontroller.Hardware.Sensors.Camera.OpenCV.VisionPipelines;

import static org.opencv.core.Core.inRange;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
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

public class SingleColorSignalFinder extends OpenCvPipeline {
    Telemetry telemetry;

    static final Scalar GREEN_COLOR = new Scalar(0, 255, 0);

    public enum Color{
        RED,
        BLUE,
        GREEN
    }

    Color color;
    boolean RED = true;
    boolean BLUE = true;
    boolean GREEN = true;

    public int redContourCount = 0;
    public int blueContourCount = 0;
    public int greenContourCount = 0;


    public List<Rect> redRect;
    public List<Rect> blueRect;
    public List<Rect> greenRect;

    public Rect RedRect;
    public Rect BlueRect;
    public Rect GreenRect;

    public List<MatOfPoint> redContours;
    public List<MatOfPoint> blueContours;
    public List<MatOfPoint> greenContours;

    public MatOfPoint biggestRedContour;
    public MatOfPoint biggestBlueContour;
    public MatOfPoint biggestGreenContour;

    public SingleColorSignalFinder(Telemetry telemetry) {
        redContours = new ArrayList<MatOfPoint>();
        redRect = new ArrayList<Rect>();
        RedRect = new Rect();
        biggestRedContour = new MatOfPoint();

        blueContours = new ArrayList<MatOfPoint>();
        blueRect = new ArrayList<Rect>();
        BlueRect = new Rect();
        biggestBlueContour = new MatOfPoint();

        greenContours = new ArrayList<MatOfPoint>();
        greenRect = new ArrayList<Rect>();
        GreenRect = new Rect();
        biggestGreenContour = new MatOfPoint();

        this.telemetry = telemetry;
    }

    // Filters the contours to be greater than a specific area in order to be tracked
    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) >250;
    }

    // Red masking thresholding values:
    Scalar lowRed = new Scalar(85, 60, 90); // 160, 170, 80 || 160, 140, 10
    Scalar highRed = new Scalar(105, 255, 255); //179, 255, 220

    // Mat object for the red mask
    Mat maskRed = new Mat();

    // Green masking thresholding values:
    Scalar lowGreen = new Scalar(30, 0, 0); //38, 0, 0
    Scalar highGreen = new Scalar(60, 240, 255); //70, 255, 255

    // Mat object for the yellow mask
    Mat maskGreen = new Mat();

    // Blue masking thresholding values:
    Scalar lowBlue = new Scalar(90, 40, 50); // 90, 160, 20
    Scalar highBlue = new Scalar(200, 255, 255); // 130, 255, 255

    // Mat object for the blue mask
    Mat maskBlue = new Mat();

    // Mat object for YCrCb color space
    Mat HSV = new Mat();

    // Kernel size for blurring
    Size kSize = new Size(5, 5);
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2 * 2) + 1, (2 * 2) + 1));

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Imgproc.erode(HSV, HSV, kernel);
        Imgproc.GaussianBlur(HSV, HSV, kSize, 0);

        if (RED) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskRed"
            inRange(HSV, lowRed, highRed, maskRed);

            // Clears the arraylists
            redContours.clear();
            redRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, redContours, -1, GREEN_COLOR); //input

            // Iterates through each contour

            maskRed.release();
        }

        if (BLUE) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskBlue"
            inRange(HSV, lowBlue, highBlue, maskBlue);

            // Clears the arraylists
            blueContours.clear();
            blueRect.clear();

            //TODO: Canny edge detection?

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, blueContours, -1, GREEN_COLOR); //input


            maskBlue.release();
        }

        if (GREEN) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskRed"
            inRange(HSV, lowGreen, highGreen, maskGreen);

            // Clears the arraylists
            greenContours.clear();
            greenRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskGreen, greenContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, greenContours, -1, GREEN_COLOR); //input

            maskGreen.release();
        }

        if (greenContours.size() > blueContours.size() && greenContours.size() > redContours.size()){
            color = Color.GREEN;
        }
        
        if (blueContours.size() > redContours.size() && blueContours.size() > greenContours.size()){
            color = Color.BLUE;
        }

        if (redContours.size() > blueContours.size() && redContours.size() > greenContours.size()){
            color = Color.RED;
        }



        switch (color){
            case RED:
                Core.bitwise_not(input, input);
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
                        color = Color.RED;

                        Imgproc.rectangle(input, RedRect, GREEN_COLOR, 2);
                    }
                }

                // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
                telemetry.addData("Red Contour ", "%7d,%7d", RedRect.x + (RedRect.width/2), RedRect.y + (RedRect.height/2));

                break;
            case BLUE:
                // Iterates through each contour
                for (int i = 0; i < blueContours.size(); i++) {
                    // Filters out contours with an area less than 50 (defined in the filter contours method)
                    if (filterContours(blueContours.get(i))) {
                        biggestBlueContour = Collections.max(blueContours, (t0, t1) -> {
                            return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                        });

                        // Creates a bounding rect around each contourand the draws it
                        BlueRect = Imgproc.boundingRect(biggestBlueContour);

                        Imgproc.rectangle(input, BlueRect, GREEN_COLOR, 2);
                    }
                }

                // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
                telemetry.addData("Blue Contour ", "%7d,%7d", BlueRect.x + (BlueRect.width/2), BlueRect.y + (BlueRect.height/2));
                break;
            case GREEN:
                // Iterates through each contour
                for (int i = 0; i < greenContours.size(); i++) {

//                Imgproc.HoughCircles(redContours.get(i), circles, Imgproc.HOUGH_GRADIENT, Math.PI/180, 50, 1, 50);
                    // Filters out contours with an area less than 50 (defined in the filter contours method)
                    if (filterContours(greenContours.get(i))) {

                        // TODO: move outside of for loop to optimize potentially
                        biggestGreenContour = Collections.max(greenContours, (t0, t1) -> {
                            return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                        });

                        // Creates a bounding rect around each contourand the draws it
                        GreenRect = Imgproc.boundingRect(biggestGreenContour);

                        Imgproc.rectangle(input, GreenRect, GREEN_COLOR, 2);
                    }
                }

                // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
                telemetry.addData("Green Contour ", "%7d,%7d", GreenRect.x + (GreenRect.width/2), GreenRect.y + (GreenRect.height/2));
        }

        redContourCount = 0;
        blueContourCount = 0;
        greenContourCount = 0;

        HSV.release();

        //TODO: move this when actually using code (this is just for EasyOpenCV sim)
        telemetry.addData("Color", color);
        telemetry.update();

        return input;
    }
}
