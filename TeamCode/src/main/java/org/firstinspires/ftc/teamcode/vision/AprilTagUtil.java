package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilTagUtil {

    private HardwareMap hardwareMap;
    private CameraConfig aprilCamera;
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline detectionPipeline;
    private Thread openingThread = null;
    private double tagSize;
    public AprilTagUtil(HardwareMap hardwareMap, CameraConfig aprilCamera, double tagSize)
    {
        this.hardwareMap=hardwareMap;
        this.aprilCamera=aprilCamera;
        this.tagSize=tagSize;
    }
    private boolean isOpening = false;
    private boolean isOpen=false;
    public boolean opening()
    {
        return isOpening;
    }
    public boolean opened()
    {
        return isOpen=false;
    }
    public void open()
    {

    }
    public int getId(int[] possibleIds)  {
        // TODO: Add while loop here
        ArrayList<AprilTagDetection> currentDetections = detectionPipeline.getLatestDetections();
        if(currentDetections!=null)
            for(AprilTagDetection detection: currentDetections) {
                for (int i = 0; i < possibleIds.length; i++) {
                    if (possibleIds[i] == detection.id) {
                        return detection.id;
                    }
                }
            }


        return 0;
    }
    public void close()
    {
        if(isOpening)
            openingThread.interrupt();
        else if(isOpen){
            new Thread(()->{
                camera.closeCameraDevice();
            }).start();
        }
    }
}
