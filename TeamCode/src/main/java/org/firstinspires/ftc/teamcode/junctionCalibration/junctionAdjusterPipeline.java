package org.firstinspires.ftc.teamcode.junctionCalibration;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.*;
import java.util.ArrayList;
import java.util.List;

@Config
public class junctionAdjusterPipeline extends OpenCvPipeline {

    public static class Results{
        public int junction_x1;
        public int junction_x2;
    };

    private static Results latestResults=new Results();
    private Scalar clearScalar = new Scalar(0, 0, 0);
    private Mat hsvInput = new Mat();
    private Mat yellowMask = new Mat();
    private Mat output = new Mat();
    private Mat ContourInput = new Mat();
    public static double lowerYellowH = 18;
    public static double lowerYellowS = 65;
    public static double lowerYellowV = 80;
    public static double upperYellowH = 31;
    public static double upperYellowS = 255;
    public static double upperYellowV = 255;

    @Override
    public Mat processFrame(Mat input) {
        hsvInput.setTo(clearScalar);
        yellowMask.setTo(clearScalar);
        output.setTo(clearScalar);

        Imgproc.cvtColor(input, hsvInput, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(hsvInput, hsvInput, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvInput, new Scalar(lowerYellowH, lowerYellowS, lowerYellowV), new Scalar(upperYellowH, upperYellowS, upperYellowV), yellowMask);
        Core.copyTo(input, output, yellowMask);

        Imgproc.morphologyEx(output, output, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(output, output, Imgproc.MORPH_CLOSE, new Mat());
        Imgproc.GaussianBlur(output, output, new Size(5.0, 15.0), 0.00);

        Imgproc.cvtColor(output, ContourInput, Imgproc.COLOR_BGR2GRAY);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(ContourInput, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect junctionRect = new Rect();
        double area = 0;

        for (int i = 0; i < contours.size(); i++) {
            Rect rectangle = Imgproc.boundingRect(contours.get(i));

            double area_sample = rectangle.width;
            if (area_sample > area) {
                area = area_sample;
                junctionRect = rectangle;
            }
        }


        Imgproc.rectangle(input, junctionRect, new Scalar(255, 255, 0));

        return input;
    }

    public  Results getLatestResults() {
        return latestResults;
    }
}
