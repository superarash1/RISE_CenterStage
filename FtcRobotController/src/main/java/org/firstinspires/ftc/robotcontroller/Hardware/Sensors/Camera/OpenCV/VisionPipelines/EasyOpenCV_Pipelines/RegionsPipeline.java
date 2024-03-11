package org.firstinspires.ftc.robotcontroller.Hardware.Sensors.Camera.OpenCV.VisionPipelines.EasyOpenCV_Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RegionsPipeline extends OpenCvPipeline {
    Telemetry telemetry;

    /** Most important section of the code: Colors **/
    static final Scalar GOLD = new Scalar(255, 215, 0);
    static final Scalar CRIMSON = new Scalar(220, 20, 60);
    static final Scalar AQUA = new Scalar(79, 195, 247);
    static final Scalar PARAKEET = new Scalar(3, 192, 74);
    static final Scalar CYAN = new Scalar(0, 139, 139);

    // Create a Mat object that will hold the color data
    Mat HSV = new Mat();

    Point REGION_TOPLEFT_ANCHOR_POINT = null;

    Point region_pointA;
    Point region_pointB;

    int REGION_WIDTH;
    int REGION_HEIGHT;

    public RegionsPipeline(int REGION_WIDTH, int REGION_HEIGHT, Telemetry telemetry) {
        this.REGION_WIDTH = REGION_WIDTH;
        this.REGION_HEIGHT = REGION_HEIGHT;

        this.telemetry = telemetry;
    }

    public RegionsPipeline(int REGION_WIDTH, int REGION_HEIGHT, Point REGION_TOPLEFT_ANCHOR_POINT, Telemetry telemetry) {
        this.REGION_WIDTH = REGION_WIDTH;
        this.REGION_HEIGHT = REGION_HEIGHT;
        this.REGION_TOPLEFT_ANCHOR_POINT = REGION_TOPLEFT_ANCHOR_POINT;

        this.telemetry = telemetry;
    }

    Size kSize = new Size(5, 5);

    // Creates a field of type "Mat"
    Mat region;

    // Creating an array for each region which have an element for each channel of interest
    public int[] channelsOfInterest = new int[3];

    @Override
    public Mat processFrame(Mat input) {

        if (REGION_TOPLEFT_ANCHOR_POINT == null){
            REGION_TOPLEFT_ANCHOR_POINT = new Point(input.cols()/2 - REGION_WIDTH/2,input.rows()/2 - REGION_HEIGHT/2);
        }

        // Create the points that will be used to make the rectangles for the region
        region_pointA = new Point(REGION_TOPLEFT_ANCHOR_POINT.x, REGION_TOPLEFT_ANCHOR_POINT.y);
        region_pointB = new Point(REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        // Converts the RGB colors from the video to HSV, which is more useful for image analysis
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        Imgproc.blur(HSV, HSV, kSize);

        // Creates the regions and finds the HSV values for each of the regions
        region = HSV.submat(new Rect(region_pointA, region_pointB));

        // Loops through each channel of interest
        for (int i = 0; i < 3; i++){
            // Finds the average HSV value for each channel of interest (The "i" representing the channel of interest)
            channelsOfInterest[i] = (int) Core.mean(region).val[i];
        }

        // Draws rectangles representing the regions in the camera stream
        Imgproc.rectangle(HSV, region_pointA, region_pointB, GOLD,1);

        telemetry.addData("Region 1", "%7d, %7d, %7d", channelsOfInterest[0], channelsOfInterest[1], channelsOfInterest[2]);
        telemetry.addData("rows", input.rows());
        telemetry.addData("columns", input.cols());

        return HSV;
    }
}
