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
import java.util.List;

public class Signal_Identifier extends OpenCvPipeline {
    Telemetry telemetry;

    // RED + GREEN, BLUE + ORANGE, YELLOW + PINK

    public enum SignalPosition {
        RIGHT,
        MIDDLE,
        LEFT,
        UNDEFINED
    }

    public boolean signalReadingToggle = true;

    public SignalPosition signalPosition = SignalPosition.UNDEFINED;

    static final Scalar GREEN = new Scalar(0, 255, 0);

    boolean topColorCheck = false;
    boolean bottomColorCheck = false;

    boolean overlapConditionOne = false;
    boolean overlapConditionTwo = false;

    boolean overlapCheckOne = false;
    boolean overlapCheckTwo = false;

    boolean distanceCheckOne = false;
    boolean distanceCheckTwo = false;

    int colorDistanceThreshold = 50;

    double maxArea;
    double previousMaxArea;
    public int maxArrayElement;

    public List<Rect> redRect;
    public List<Rect> filteredRect;
    public List<Rect> greenRect;

    public List<Rect> blueRect;
    public List<Rect> filteredBlueRect;
    public List<Rect> orangeRect;

    public List<Rect> yellowRect;
    public List<Rect> filteredYellowRect;
    public List<Rect> pinkRect;

    public List<MatOfPoint> redContours;
    public List<MatOfPoint> greenContours;
    public List<MatOfPoint> blueContours;
    public List<MatOfPoint> orangeContours;
    public List<MatOfPoint> yellowContours;
    public List<MatOfPoint> pinkContours;

    public Rect TargetRect;

    public Signal_Identifier(Telemetry telemetry) {
        redContours = new ArrayList<MatOfPoint>();
        redRect = new ArrayList<Rect>();
        filteredRect = new ArrayList<Rect>();

        blueContours = new ArrayList<MatOfPoint>();
        blueRect = new ArrayList<Rect>();
        filteredBlueRect = new ArrayList<Rect>();

        yellowContours = new ArrayList<MatOfPoint>();
        yellowRect = new ArrayList<Rect>();
        filteredYellowRect = new ArrayList<Rect>();

        greenContours = new ArrayList<MatOfPoint>();
        greenRect = new ArrayList<Rect>();

        orangeContours = new ArrayList<MatOfPoint>();
        orangeRect = new ArrayList<Rect>();

        pinkContours = new ArrayList<MatOfPoint>();
        pinkRect = new ArrayList<Rect>();

        TargetRect = new Rect();

        this.telemetry = telemetry;
    }

    // Filters the contours to be greater than a specific area in order to be tracked
//    public boolean filterContours(MatOfPoint contour) {
//        return Imgproc.contourArea(contour) > 50;
//    }

    // Red masking thresholding values:
    Scalar lowRed = new Scalar(85, 60, 50); // 160, 170, 80 || 160, 140, 10
    Scalar highRed = new Scalar(115, 255, 255); //179, 255, 220

    // Mat object for the red mask
    Mat maskRed = new Mat();

    // Green masking thresholding values:
    Scalar lowGreen = new Scalar(30, 50, 20); //38, 0, 0
    Scalar highGreen = new Scalar(60, 240, 255); //70, 255, 255

    // Mat object for the yellow mask
    Mat maskGreen = new Mat();

    // Blue masking thresholding values:
    Scalar lowBlue = new Scalar(90, 40, 50); // 90, 160, 20
    Scalar highBlue = new Scalar(200, 255, 255); // 130, 255, 255

    // Mat object for the blue mask
    Mat maskBlue = new Mat();

    // Orange masking thresholding values:
    Scalar lowOrange = new Scalar(10, 50, 50); // 160, 170, 80 || 160, 170, 0
    Scalar highOrange = new Scalar(30, 255, 255); //180, 240, 220

    // Mat object for the orange mask
    Mat maskOrange = new Mat();

    // Yellow masking thresholding values:
    Scalar lowYellow = new Scalar(20, 100, 50); //180
    Scalar highYellow = new Scalar(35, 255, 255);

    // Mat object for the yellow mask
    Mat maskYellow = new Mat();

    // Pink masking thresholding values:
    Scalar lowPink = new Scalar(140, 10, 10); // 160, 10, 225
    Scalar highPink = new Scalar(179, 255, 255); // 180, 255, 255

    // Mat object for the pink mask
    Mat maskPink = new Mat();

    // Mat object for HSV color space
    Mat HSV = new Mat();
    Mat HSV_inv = new Mat();

    // Kernel size for blurring
    Size kSize = new Size(5, 5);
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2 * 2) + 1, (2 * 2) + 1));

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Imgproc.erode(HSV, HSV, kernel);

        // Finds the pixels within the thresholds and puts them in the mat object

        Core.bitwise_not(input, input);

        Imgproc.cvtColor(input, HSV_inv, Imgproc.COLOR_RGB2HSV);
        Imgproc.erode(HSV_inv, HSV_inv, kernel);

        inRange(HSV_inv, lowRed, highRed, maskRed);
        inRange(HSV, lowGreen, highGreen, maskGreen);
        inRange(HSV, lowBlue, highBlue, maskBlue);
        inRange(HSV, lowOrange, highOrange, maskOrange);
        inRange(HSV, lowYellow, highYellow, maskYellow);
        inRange(HSV, lowPink, highPink, maskPink);

        // Clears the arraylists
        redContours.clear();
        redRect.clear();

        greenContours.clear();
        greenRect.clear();

        blueContours.clear();
        blueRect.clear();

        orangeContours.clear();
        orangeRect.clear();

        yellowContours.clear();
        yellowRect.clear();

        pinkContours.clear();
        pinkRect.clear();

        // Finds the contours and draws them on the screen
        Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskGreen, greenContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskOrange, orangeContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskYellow, yellowContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskPink, pinkContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Iterates through each contour
        for (int i = 0; i < redContours.size(); i++) {
            // Creates a bounding rect around each contour and the draws it
            redRect.add(Imgproc.boundingRect(redContours.get(i)));
        }

        for (int i = 0; i < greenContours.size(); i++) {
            // Creates a bounding rect around each contour and the draws it
            greenRect.add(Imgproc.boundingRect(greenContours.get(i)));
        }

        for (int i = 0; i < blueContours.size(); i++) {
            // Creates a bounding rect around each contour and the draws it
            blueRect.add(Imgproc.boundingRect(blueContours.get(i)));
        }

        for (int i = 0; i < orangeContours.size(); i++) {
            // Creates a bounding rect around each contour and the draws it
            orangeRect.add(Imgproc.boundingRect(orangeContours.get(i)));
        }

        for (int i = 0; i < yellowContours.size(); i++) {
            // Creates a bounding rect around each contour and the draws it
            yellowRect.add(Imgproc.boundingRect(yellowContours.get(i)));
        }

        for (int i = 0; i < pinkContours.size(); i++) {
            // Creates a bounding rect around each contour and the draws it
            pinkRect.add(Imgproc.boundingRect(pinkContours.get(i)));
        }

        filteredRect.clear();

        if (signalReadingToggle){
            readSignal();
        }

        Imgproc.rectangle(input, TargetRect, GREEN, 2);

        maskRed.release();
        maskGreen.release();
        maskBlue.release();
        maskOrange.release();
        maskYellow.release();
        maskPink.release();

        filteredRect.clear();
        filteredBlueRect.clear();
        filteredYellowRect.clear();

        HSV.release();

        Core.bitwise_not(input, input);
        return input;
    }

    public void readSignal(){
        if (isMarkerPresent(yellowRect, pinkRect)){
            signalPosition = SignalPosition.RIGHT;
        } else if (isMarkerPresent(redRect, greenRect)) {
            signalPosition = SignalPosition.LEFT;
        } else if (isMarkerPresent(blueRect, orangeRect)){
            signalPosition = SignalPosition.MIDDLE;
        }
    }

    public int getRectCenterX(Rect rect){
        return rect.x + (rect.width/2);
    }

    public int getRectCenterY(Rect rect){
        return rect.y + (rect.height/2);
    }

    public boolean isMarkerPresent(List<Rect> innerColor, List<Rect> outerColor) {
        if (!innerColor.isEmpty()){
            for (Rect innerColors : innerColor){
                if (!outerColor.isEmpty()){
                    for (Rect outerColors : outerColor) {
                        if (getRectCenterY(outerColors) > getRectCenterY(innerColors)) {
                            topColorCheck = true;
                            overlapCheckOne = true;
                        }

                        if (getRectCenterY(outerColors) < getRectCenterY(innerColors)){
                            bottomColorCheck = true;
                            overlapCheckTwo = true;
                        }
///
                        if ((outerColors.x < getRectCenterX(innerColors)) && getRectCenterX(innerColors) < outerColors.x + outerColors.width && overlapCheckOne) {
                            overlapConditionOne = true;
                        }

                        if (outerColors.x < getRectCenterX(innerColors) && getRectCenterX(innerColors) < outerColors.x + outerColors.width && overlapCheckTwo) {
                            overlapConditionTwo = true;
                        }

                        if ((innerColors.y - outerColors.y + outerColors.height < colorDistanceThreshold) && overlapCheckOne){
                            distanceCheckOne = true;
                        }

                        if ((outerColors.y + outerColors.height - innerColors.y < colorDistanceThreshold) && overlapCheckTwo){
                            distanceCheckTwo = true;
                        }

                        overlapCheckOne = false;
                        overlapCheckTwo = false;
                    }
                }

                if (topColorCheck && bottomColorCheck && overlapConditionOne && overlapConditionTwo && distanceCheckOne && distanceCheckTwo){
                    filteredRect.add(innerColors);
                }

                topColorCheck = false;
                bottomColorCheck = false;
                overlapConditionOne = false;
                overlapConditionTwo = false;
                distanceCheckOne = false;
                distanceCheckTwo = false;
            }
        }

        maxArea = 0;
        previousMaxArea = 0;
        maxArrayElement = 0;

//        for (int i = 0; i < (filteredRect.size()); i++){
//            if (filteredRect.get(i).area() > maxArea){
//                maxArea = filteredRect.get(i).area();
//                TargetRect = filteredRect.get(i);
//            }
////            if (maxArea > previousMaxArea) maxArrayElement = i;
////
////
////            previousMaxArea = maxArea;
//        }

        if (!filteredRect.isEmpty()){
            for (Rect filteredRects : filteredRect){
                if (filteredRects.area() > maxArea){
                    maxArea = filteredRects.area();
                    TargetRect = filteredRects;
                }
            }
        }


        if (!TargetRect.empty()){
//            TargetRect = filteredRect.get(maxArrayElement);
            return true;
        } else {
            return false;
        }
    }
}
