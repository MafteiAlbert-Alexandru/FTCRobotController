package org.firstinspires.ftc.teamcode.junctionCalibration;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.*;
@Config
public class junctionAdjusterPipeline extends OpenCvPipeline
{
    public static class Results{
        public int junction_x1;
        public int junction_x2;

        long time;

        boolean wasAccesed;
    };

    private final double treshold = 1;       //trebuie ajustat
    private final double[] newPixel = {1,1,1}; // pentru vizualizare
    private static Results latestResults;
    private Mat hsvInput = new Mat();
    private Mat yellowMask = new Mat();
    private Mat output = new Mat();
    public static double lowerYellowH=45;
    public static double lowerYellowS=0;
    public static double lowerYellowV=0;
    public static double upperYellowH=72;
    public static double upperYellowS=255;
    public static double upperYellowV=255;
    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, hsvInput, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(hsvInput, hsvInput, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvInput, new Scalar(lowerYellowH, lowerYellowS, lowerYellowV), new Scalar(upperYellowH, upperYellowS, upperYellowV), yellowMask);

        Core.copyTo(input, output, yellowMask);

        return output;
        /*
        int maxWidth = 0;
        int maxPole_x = 0;

        int y = input.height()/2;
        int width = 0;
        int pole_x = 0;
        boolean isJunction = false;
        double[] firstPixel = input.get(y, 0);
        double lastYellow = firstPixel[0] + firstPixel[1];

        for(int x = 1; x < input.width(); x++){
            double[] pixel = input.get(y, x);
            double yellow = pixel[0] + pixel[1];
            double diff = Math.abs(yellow - lastYellow);
            if(diff > treshold){
                if(yellow - lastYellow < 0){
                    isJunction = true;
                    width = 0;
                    pole_x = x;
                }else{
                    isJunction = false;
                    if(width > maxWidth){
                        maxWidth = width;
                        maxPole_x = pole_x;
                    }
                }
            }
            if(isJunction){
                width++;
                input.put(y,x,newPixel);
            }
        }

        latestResults.junction_x1 = maxPole_x;
        latestResults.junction_x2 = maxPole_x + maxWidth;
        latestResults.time = System.nanoTime();
        latestResults.wasAccesed = false;
        return input;*/
    }


    public static Results getLatestResults(){
        return latestResults;
    }
}
