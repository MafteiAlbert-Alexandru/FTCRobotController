package org.firstinspires.ftc.teamcode.vision.pipeline;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.*;

class junctionAdjusterPipeline extends OpenCvPipeline
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

    @Override
    public Mat processFrame(Mat input)
    {
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
        return input;
    }


    public static Results getLatestResults(){
        return latestResults;
    }
}
