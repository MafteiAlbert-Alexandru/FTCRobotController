package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilTagUtil {

//    private HardwareMap hardwareMap;
    private CameraConfig aprilCamera = CameraConfig.C920;
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline detectionPipeline;
    private Thread openingThread = null;
    private double tagSize = 0.166;
    LinearOpMode opMode;
    public SampleMecanumDriveCancelable sampleMecanumDriveCancelable;
    public AprilTagUtil(LinearOpMode opMode)
    {
        this.opMode = opMode;
        open();
    }

    public AprilTagUtil(LinearOpMode opMode, SampleMecanumDriveCancelable driveCancelable)
    {
        this.opMode = opMode;
        sampleMecanumDriveCancelable = driveCancelable;
        open();
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
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        detectionPipeline = new AprilTagDetectionPipeline(tagSize, CameraConfig.C920.getFx(), CameraConfig.C920.getFy(), CameraConfig.C920.getCx(), CameraConfig.C920.getCy());

        FtcDashboard.getInstance().startCameraStream(camera, 10);

        camera.setPipeline(detectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }
    public int getId()  {
        int[] possibleIds = new int[]{1,2,3};

        // TODO: Add while loop here
        while(true)
        {
            ArrayList<AprilTagDetection> currentDetections = detectionPipeline.getLatestDetections();
            if(currentDetections!=null)
                for(AprilTagDetection detection: currentDetections) {
                    for (int i = 0; i < possibleIds.length; i++) {
                        if (possibleIds[i] == detection.id) {
                            int id = detection.id;
                            close();
                            return id;
                        }
                    }
                }
        }
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
