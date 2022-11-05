package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.junctionCalibration.junctionAdjusterPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.junctionCalibration.junctionAdjuster;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    int cameraMonitorViewId;
    OpenCvCamera webcam;

    junctionAdjuster j_adjuster;

    junctionAdjuster.CameraData Cam;
    junctionAdjuster.JunctionData Junction;
    junctionAdjuster.MecanumResources MecanumR;

    junctionAdjusterPipeline pipeline;


    @Override
    public void runOpMode(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{


        Hardware_Map drive = new Hardware_Map(hardwareMap);

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));

        pipeline = new junctionAdjusterPipeline();



            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    webcam.setPipeline(pipeline);
                    FtcDashboard.getInstance().startCameraStream(webcam, 0);
                }
                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
        //Cam.FOV_x = 1;              //
        //Cam.resolution_x = 320;     // trb ajustat
        //Cam.resolution_y = 240;     //

        //Junction.diameter = 2.54;   //cm

        //j_adjuster = new junctionAdjuster(webcam, Cam, Junction, MecanumR);

            //camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

        waitForStart();
        //OpenCamDevice();
        while(opModeIsActive()&&!isStopRequested()){
        }
        }catch (Exception e)
        {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }

    private void OpenCamDevice(){
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                // Usually this is where you'll want to start streaming from the camera (see section 4)
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }
}