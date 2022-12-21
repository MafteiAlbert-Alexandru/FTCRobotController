//package org.firstinspires.ftc.teamcode.junctionCalibration;
//
//import com.acmerobotics.dashboard.config.Config;
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.MatOfPoint2f;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.RotatedRect;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.lang.*;
//import java.util.ArrayList;
//import java.util.List;
//
//@Config
//public class junctionAdjusterPipeline extends OpenCvPipeline {
//
//    public class Results{
//        public int junction_x1;
//        public int junction_x2;
//        public double area;
//    };
//
//    private Results latestResults=new Results();
//    private Scalar clearScalar = new Scalar(0, 0, 0);
//    private Mat hsvInput = new Mat();
//    private Mat yellowMask = new Mat();
//    private Mat output = new Mat();
//    private Mat ContourInput = new Mat();
//    public static double lowerYellowH = 12;
//    public static double lowerYellowS = 90;
//    public static double lowerYellowV = 40;
//    public static double upperYellowH = 35;
//    public static double upperYellowS = 255;
//    public static double upperYellowV = 255;
//
//    @Override
//    public Mat processFrame(Mat input) {
//        hsvInput.setTo(clearScalar);
//        yellowMask.setTo(clearScalar);
//        output.setTo(clearScalar);
//
//        Imgproc.cvtColor(input, hsvInput, Imgproc.COLOR_RGBA2RGB);
//        Imgproc.cvtColor(hsvInput, hsvInput, Imgproc.COLOR_RGB2HSV);
//
//        Core.inRange(hsvInput, new Scalar(lowerYellowH, lowerYellowS, lowerYellowV), new Scalar(upperYellowH, upperYellowS, upperYellowV), yellowMask);
//        Core.copyTo(input, output, yellowMask);
//
//        //Imgproc.morphologyEx(output, output, Imgproc.MORPH_OPEN, new Mat());
//        //Imgproc.morphologyEx(output, output, Imgproc.MORPH_CLOSE, new Mat());
//        //Imgproc.GaussianBlur(output, output, new Size(5.0, 15.0), 0.00);
//
//        Imgproc.cvtColor(output, ContourInput, Imgproc.COLOR_BGR2GRAY);
//        List<MatOfPoint> contours = new ArrayList<>();
//        Imgproc.findContours(ContourInput, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        RotatedRect junctionRect = new RotatedRect();
//        double area = 0;
//
//        for (int i = 0; i < contours.size(); i++) {
//            MatOfPoint2f NewMtx = new MatOfPoint2f( contours.get(i).toArray() );
//            Imgproc.drawContours(input, contours, i, new Scalar(0,0,255));
//            RotatedRect rectangle = Imgproc.minAreaRect(NewMtx);
//
//            double area_sample = Math.min(rectangle.size.width, rectangle.size.height);
//            if (area_sample > area) {
//                area = area_sample;
//                junctionRect = rectangle;
//            }
//
//            NewMtx.release();
//        }
//
//        Point[] points = new Point[4];
//        junctionRect.points(points);
//
//        double mid_x = (points[0].x + points[2].x)/2;
//
//        latestResults.junction_x1 = (int)(mid_x - area/2);
//        latestResults.junction_x2 = (int)(mid_x + area/2);
//        latestResults.area = area * Math.max(junctionRect.size.width, junctionRect.size.height);
//
//        Imgproc.line(input, points[0], points[1], new Scalar(255, 255, 0));
//        Imgproc.line(input, points[1], points[2], new Scalar(255, 255, 0));
//        Imgproc.line(input, points[2], points[3], new Scalar(255, 255, 0));
//        Imgproc.line(input, points[3], points[0], new Scalar(255, 255, 0));
//
//
//        return input;
//    }
//
//    public  Results getLatestResults() {
//        return latestResults;
//    }
//}
