//package org.firstinspires.ftc.teamcode.junction;
//
//import android.os.Environment;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
//
//import java.io.File;
//import java.io.FileInputStream;
//import java.io.IOException;
//import java.util.concurrent.ExecutorService;
//import java.util.concurrent.Executors;
//
//public class LUT {
//
//    private static long ptrToLut=0;
//    private static boolean alreadyLoaded = false;
//    public static ExecutorService threadPool = Executors.newFixedThreadPool(4);
//    private native static long createPtrToLut(byte[] data);
//    @OpModeRegistrar
//    public static void loadLUT() throws IOException {
//
//        if(alreadyLoaded)
//            return;
//        byte[] data = new byte[256*256*256];
//        File lutFile = new File(String.format("%s/FIRST/%s", Environment.getExternalStorageDirectory(), "LUT.dat"));
//        if(lutFile.exists())
//        {
//            FileInputStream inputStream = new FileInputStream(lutFile);
//            inputStream.read(data, 0, 256*256*256);
//            inputStream.close();
//        }
//        alreadyLoaded=true;
//        ptrToLut=createPtrToLut(data);
//    }
//
//
//    public static native void lutOperation(long inputPtr, long outputPtr);
//
//}
