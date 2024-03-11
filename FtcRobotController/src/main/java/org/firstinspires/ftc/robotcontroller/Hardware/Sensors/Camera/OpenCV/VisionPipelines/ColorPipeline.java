package org.firstinspires.ftc.robotcontroller.Hardware.Sensors.Camera.OpenCV.VisionPipelines;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorPipeline extends OpenCvPipeline {

    private static final Point TOPRIGHT = new Point(200, 200);
    private static final Point BOTTOMLEFT = new Point(100, 100);

    private static final Scalar WHITE = new Scalar(255, 255, 255);

    public enum Colors{
        RED,
        GREEN,
        BLUE,
        UNKNOWN
    }

    private Mat Region;
    private Mat HSV = new Mat();
    private static Scalar avg;
    private double hue;
    private static String color;
    public Telemetry telemetry;

    public ColorPipeline (Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void init(){

    }
    public int[] channelsOfInterest = new int[3];

    @Override
    public Mat processFrame(Mat input) {

        // Define the dimensions and location of each region
        Point REGION_TOPLEFT_ANCHOR_POINT = new Point((input.cols()/2)-100,(input.rows()/2) + 25);
        int REGION_WIDTH = 10;
        int REGION_HEIGHT = 3;

        //TODO: -5
        // Create the points that will be used to make the rectangles for the region
        Point region_pointA = new Point(REGION_TOPLEFT_ANCHOR_POINT.x, REGION_TOPLEFT_ANCHOR_POINT.y);
        Point region_pointB = new Point(REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        // Converts the RGB colors from the video to HSV, which is more useful for image analysis
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        Size kSize = new Size(5, 5);

        //Imgproc.blur(HSV, HSV, kSize);
        // Creates the regions and finds the HSV values for each of the regions
        Region = HSV.submat(new Rect(region_pointA, region_pointB));

        hue = Core.mean(Region).val[0];

        Imgproc.rectangle(HSV, region_pointA, region_pointB, WHITE,1);

//        telemetry.addData("color", findColor());
//        telemetry.addData("Hue", getHue());
//
//        telemetry.update();
        return HSV;
    }

    public Colors findColor() {
        if ((hue >= 150 && hue <= 179) || (hue <= 7 && hue >= 0))
            return Colors.RED;
        else if (hue >= 50 && hue <= 90)
            return Colors.GREEN;
        else if (hue >= 100 && hue <= 135)
            return Colors.BLUE;
        else
            return Colors.UNKNOWN;
    }

    //@return: the hue!!
    public double getHue() {
        return hue;
    }
}

