package org.firstinspires.ftc.teamcode.junctionCalibration;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class LUT {
    public static byte[] data = new byte[256*256*256];
    private static boolean lutFound = false;
    private static boolean alreadyLoaded = false;
    public static ExecutorService threadPool = Executors.newFixedThreadPool(4);
    @OpModeRegistrar
    public static void loadLUT() throws IOException {

        if(alreadyLoaded)
            return;


        File lutFile = new File(String.format("%s/FIRST/%s", Environment.getExternalStorageDirectory(), "LUT.dat"));
        if(lutFile.exists())
        {
            lutFound=true;
            FileInputStream inputStream = new FileInputStream(lutFile);
            inputStream.read(data, 0, 256*256*256);
            inputStream.close();
        }
        alreadyLoaded=true;
    }


    public static void lutOperation(Mat input, Mat output) throws InterruptedException {
        final int  cols = input.cols();
        final int rows = input.rows();
        /*Future[] futures = new Future[4];
        for(int i=0;i<4;i++)
        {
            int finalI = i;
            futures[i]=threadPool.submit(()-> {
                for(int y = cols/4* finalI; y<cols/4*(finalI +1); y++)
                {
                    for(int x =0; x<rows; x++)
                    {
                        double[] inputPixel = input.get(x,y);
                        output.put(x,y, data[((int)inputPixel[0])*256*256+((int)inputPixel[1])*256+((int)inputPixel[2])]);
                    }
                }
            });
        }
        for(int i=0;i<4;i++) futures[i].wait();*/

        for(int y = 0; y<rows; y++)
        {
            for(int x =0; x<cols; x++)
            {
                double[] inputPixel = input.get(x,y);
                output.put(x,y, (double)data[((int)inputPixel[0])*256*256+((int)inputPixel[1])*256+((int)inputPixel[2])] * 255.0);
            }
        }
    }

}
