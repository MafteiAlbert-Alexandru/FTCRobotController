//package org.firstinspires.ftc.teamcode.junctionCalibration;
//
//import com.acmerobotics.dashboard.config.Config;
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.MatOfPoint2f;
//import org.opencv.core.Point;
//import org.opencv.core.RotatedRect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//@Config
//public class PixelJunctionAdjusterPipeline extends OpenCvPipeline {
//    public static class Results {
//        public double centerPoint;
//        public double width;
//        public double angle;
//    }
//
//    public static double lowerYellowH = 18;
//    public static double lowerYellowS = 90;
//    public static double lowerYellowV = 40;
//    public static double upperYellowH = 35;
//    public static double upperYellowS = 255;
//    public static double upperYellowV = 255;
//    public static double threshold = 100;
//    public static double realWeight=0.05;
//
//    public Mat hsvInput=new Mat();
//    public Mat yellowMask = new Mat();
//    public Mat output;
//    private final Mat contourInput = new Mat();
//    private final Object resultsLock = new Object();
//    private Results latestResults = null;
//
//    @Override
//    public Mat processFrame(Mat input) {
//        output=new Mat();
//        // Initial image conversion
//        Imgproc.cvtColor(input, hsvInput, Imgproc.COLOR_RGBA2RGB);
//        Imgproc.cvtColor(hsvInput, hsvInput, Imgproc.COLOR_RGB2HSV);
//
//        // Masking of yellow pixels
//        Core.inRange(hsvInput, new Scalar(lowerYellowH, lowerYellowS, lowerYellowV), new Scalar(upperYellowH, upperYellowS, upperYellowV), yellowMask);
//        Core.copyTo(input, output, yellowMask);
//
//        // Conversion of masked output to grayscale
//        Imgproc.cvtColor(output, contourInput, Imgproc.COLOR_BGR2GRAY);
//
//        // Contour detection
//        List<MatOfPoint> contours = new ArrayList<>();
//        Imgproc.findContours(contourInput, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        RotatedRect junctionRect = null;
//        double width = 0;
//
//        // Finds the largest minAreaRectangle larger than a certain threshold
//        for (int i = 0; i < contours.size(); i++) {
//
//            MatOfPoint2f NewMtx = new MatOfPoint2f( contours.get(i).toArray() );
//            Imgproc.drawContours(input, contours, i, new Scalar(0,0,255));
//            RotatedRect rectangle = Imgproc.minAreaRect(NewMtx);
//
//            double realArea = Imgproc.contourArea(contours.get(i));
//            if(realArea>threshold)
//            {
//                // We do this for poles with noise next to them
//                double maximumDimension = Math.max(rectangle.size.width, rectangle.size.height);
//                double realWidth = realArea/maximumDimension;
//
//                double rectangleWidth= realWidth*realWeight + (1-realWeight)*Math.min(rectangle.size.width, rectangle.size.height);
//
//                if (rectangleWidth > width) {
//                    width = rectangleWidth;
//                    junctionRect = rectangle;
//                }
//                NewMtx.release();
//            }
//
//        }
//        if(junctionRect!=null)
//        {
//            Point[] points = new Point[4];
//            junctionRect.points(points);
//            if(latestResults==null) latestResults=new Results();
//            synchronized (resultsLock)
//            {
//
//                latestResults.centerPoint = junctionRect.center.x;
//                latestResults.width = width;
//                latestResults.angle = junctionRect.angle;
//            }
//
//            Imgproc.line(input, points[0], points[1], new Scalar(255, 255, 0));
//            Imgproc.line(input, points[1], points[2], new Scalar(255, 255, 0));
//            Imgproc.line(input, points[2], points[3], new Scalar(255, 255, 0));
//            Imgproc.line(input, points[3], points[0], new Scalar(255, 255, 0));
//        }
//        output.release();
//        return input;
//    }
//
//    public Results getLatestResults()
//    {
//        synchronized (resultsLock)
//        {
//            return latestResults;
//        }
//    }
//
//
//
//
//
//}
