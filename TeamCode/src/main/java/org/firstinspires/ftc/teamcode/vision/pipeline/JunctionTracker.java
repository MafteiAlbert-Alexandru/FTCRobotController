package org.firstinspires.ftc.teamcode.vision.pipeline;

import static org.opencv.core.Core.inRange;

import com.acmerobotics.roadrunner.geometry.Vector2d;

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

public class JunctionTracker extends OpenCvPipeline {
    Telemetry telemetry;

    static final Scalar RED_Rect = new Scalar(255, 255, 0);
    static final Scalar GREEN_Rect = new Scalar(0, 0, 0);
    static final Scalar BLUE_Rect = new Scalar(0, 0, 255);
    static final Scalar YELLOW_Rect = new Scalar(0, 0, 0);

    boolean RED = true;
    boolean BLUE = true;
    boolean YELLOW = true;

    public int redContourCount = 0;
    public int blueContourCount = 0;
    public int yellowContourCount = 0;

    public List<Rect> redRect;
    public List<Rect> blueRect;
    public List<Rect> yellowRect;

    public Rect[] rectArray;

    public Rect RedRect;
    public Rect BlueRect;
    public Rect YellowRect;

    public List<MatOfPoint> redContours;
    public List<MatOfPoint> blueContours;
    public List<MatOfPoint> yellowContours;

    public MatOfPoint biggestRedContour;
    public MatOfPoint biggestBlueContour;
    public MatOfPoint biggestYellowContour;

    public JunctionTracker(Telemetry telemetry) {
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
        return Imgproc.contourArea(contour) > 50;
    }

    // Red masking thresholding values:
    Scalar lowRed = new Scalar(0, 160, 0); //10, 100, 50
    Scalar highRed = new Scalar(200, 200, 200); //35, 255, 255

    // Blue masking thresholding values:
    Scalar lowBlue = new Scalar(0, 0, 140); //10, 100, 50
    Scalar highBlue = new Scalar(175, 200, 200); //35, 255, 255

    Scalar lowYellow = new Scalar(0, 0, 0);
    Scalar highYellow = new Scalar(0, 0, 0);

    // Mat object for the red and blue mask
    Mat maskRed = new Mat();
    Mat maskBlue = new Mat();
    Mat maskYellow = new Mat();

    // Mat object for YCrCb color space
    Mat YCrCb = new Mat();

    // Kernel size for blurring
    Size kSize = new Size(5, 5);
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2 * 2) + 1, (2 * 2) + 1));

    public Vector2d junctionCenter;
    public Rect junctionRect;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(YCrCb, YCrCb, kernel);

        if (RED) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskRed"
            inRange(YCrCb, lowRed, highRed, maskRed);

            // Clears the arraylists
            redContours.clear();
            redRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, redContours, -1, RED_Rect); //input

            // Iterates through each contour
            for (int i = 0; i < redContours.size(); i++) {

                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(redContours.get(i))) {
                    biggestRedContour = Collections.max(redContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contourand the draws it
                    RedRect = Imgproc.boundingRect(biggestRedContour);

                    Imgproc.rectangle(input, RedRect, RED_Rect, 2);
                }
            }

            // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
            telemetry.addData("Red Contour ", "%7d,%7d", RedRect.x + (RedRect.width / 2), RedRect.y + (RedRect.height / 2));

            maskRed.release();
        }

        if (BLUE) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskBlue"
            inRange(YCrCb, lowBlue, highBlue, maskBlue);

            // Clears the arraylists
            blueContours.clear();
            blueRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, blueContours, -1, BLUE_Rect); //input

            // Iterates through each contour
            for (int i = 0; i < blueContours.size(); i++) {
                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(blueContours.get(i))) {
                    biggestBlueContour = Collections.max(blueContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contourand the draws it
                    BlueRect = Imgproc.boundingRect(biggestBlueContour);
                    Imgproc.rectangle(input, BlueRect, BLUE_Rect, 2);
                }
            }

            // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
            telemetry.addData("Blue Contour ", "%7d,%7d", BlueRect.x + (BlueRect.width / 2), BlueRect.y + (BlueRect.height / 2));

            maskBlue.release();
        }

        if (YELLOW) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskBlue"
            inRange(YCrCb, lowYellow, highYellow, maskYellow);

            // Clears the arraylists
            yellowContours.clear();
            blueRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskYellow, yellowContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, yellowContours, -1, YELLOW_Rect); //input

            // Iterates through each contour
            for (int i = 0; i < yellowContours.size(); i++) {
                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(yellowContours.get(i))) {
                    biggestYellowContour = Collections.max(yellowContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    YellowRect = Imgproc.boundingRect(biggestYellowContour);
                    Imgproc.rectangle(input, YellowRect, YELLOW_Rect, 2);
                }
            }

            telemetry.addData("Yellow Contour ", "%7d,%7d", YellowRect.x + (YellowRect.width / 2), YellowRect.y + (YellowRect.height / 2));

            maskYellow.release();
        }

        junctionCenter = CenterOfRect(getJunctionRect());
        junctionRect = getJunctionRect();

        redContourCount = 0;
        blueContourCount = 0;
        yellowContourCount = 0;

        YCrCb.release();

        return input;
    }

    private Vector2d CenterOfRect(Rect rect){
        return new Vector2d(rect.x + ((double)rect.width / 2), rect.y + ((double)rect.height / 2));
    }

    public Rect getJunctionRect(){
        float area =  Math.max(Math.max(YellowRect.x * YellowRect.y, BlueRect.x * BlueRect.y), RedRect.x * RedRect.y);
        if(area == RedRect.x * RedRect.y) return RedRect;
        else if(area == BlueRect.x * BlueRect.y) return BlueRect;
        else return YellowRect;
    }

    public Vector2d getJunctionCenter(){ return junctionCenter;}

//    public Vector2d VectorOfJunction(){
//        for (int i = 0; i < yellowContours.size() + redContours.size() + blueContours.size(); i++) {
//            // Filters out contours with an area less than 50 (defined in the filter contours method)
//            if (filterContours(yellowContours.get(i))) {
//                biggestYellowContour = Collections.max(yellowContours, (t0, t1) -> {
//                    return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
//                });
//
//                YellowRect = Imgproc.boundingRect(biggestYellowContour);
//                Imgproc.rectangle(input, YellowRect, YELLOW_Rect, 2);
//            }
//        }
//    }
}