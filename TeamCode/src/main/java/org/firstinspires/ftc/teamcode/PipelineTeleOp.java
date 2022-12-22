package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.junctionCalibration.JunctionAdjuster;
import org.firstinspires.ftc.teamcode.junctionCalibration.junctionAdjusterPipeline;
import org.firstinspires.ftc.teamcode.vision.WebcamUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class PipelineTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{
            OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));
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
            junctionAdjusterPipeline pipeline = new junctionAdjusterPipeline();
            webcam.setPipeline(pipeline);


            waitForStart();
            FtcDashboard.getInstance().startCameraStream(webcam, 0);
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
