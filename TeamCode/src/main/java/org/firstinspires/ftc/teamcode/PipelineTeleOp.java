package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.junction.LUT;
import org.firstinspires.ftc.teamcode.junction.JunctionAdjusterPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class PipelineTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        try{
            LUT.loadLUT();
            OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                    telemetry.addData("initial camera exposure",webcam.getExposureControl().getExposure(TimeUnit.MICROSECONDS));
                    webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
                }

                @Override
                public void onError(int errorCode) {

                }
            });
            JunctionAdjusterPipeline pipeline = new JunctionAdjusterPipeline();
            webcam.setPipeline(pipeline);


            waitForStart();
//            FtcDashboard.getInstance().startCameraStream(webcam, 0);
            while (opModeIsActive()&&!isStopRequested()){

            }

            webcam.stopStreaming();
            webcam.closeCameraDevice();

        }catch(Exception e)
        {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }
}
