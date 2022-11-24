package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.CameraState;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilTagUtil {

    private HardwareMap hardwareMap;
    private AprilCamera aprilCamera;
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline detectionPipeline;
    private Thread openingThread = null;
    private double tagSize;
    public AprilTagUtil(HardwareMap hardwareMap, AprilCamera aprilCamera, double tagSize)
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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        detectionPipeline=new AprilTagDetectionPipeline(aprilCamera.getFx(), aprilCamera.getFy(), aprilCamera.getCx(), aprilCamera.getCy(), 0.1666);
        camera.setPipeline(detectionPipeline);
        openingThread = new Thread(()->{
            try {
                int result = -1;
                while (result < 0) {
                    isOpening = true;
                    result = camera.openCameraDevice();
                    Thread.sleep(10);
                }
                isOpen = true;
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }catch(InterruptedException ignored)
            {
                isOpening=false;
                isOpen=false;
            }
        });
        openingThread.start();

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
