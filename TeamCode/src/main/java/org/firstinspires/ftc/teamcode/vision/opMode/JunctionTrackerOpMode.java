package org.firstinspires.ftc.teamcode.vision.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipeline.JunctionTracker;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class JunctionTrackerOpMode extends LinearOpMode {

    // Define Webcam
    OpenCvCamera webcam;

    // Create pipeline
    static JunctionTracker pipeline;

    private ElapsedTime runtime = new ElapsedTime();

    Vector2d junction;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FtcDashboard dashboard;

        // Set up camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        // Set up pipeline
        pipeline = new JunctionTracker(telemetry);
        webcam.setPipeline(pipeline);

        // Start camera streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        waitForStart();

        while (opModeIsActive()) {



            telemetry.update();
        }
    }

    void JunctionPID(){
        junction = pipeline.getJunctionCenter();

        double errorHeading = junction.getX();

    }


}