package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
@Config
public class WebcamUtil {
    private Servo webcamServo;
    private OpenCvWebcam webcam;
    private CameraConfig config;
    private ArrayList<WebcamUtilsListener> listeners= new ArrayList<>();
    private Telemetry telemetry;
    public static int exposureTime_microseconds = 17500;
    public WebcamUtil(HardwareMap hardwareMap, Telemetry telemetry_)
    {
        config=CameraConfig.C920;
        this.telemetry = telemetry_;
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));
        webcamServo=hardwareMap.get(Servo.class, "webcamServo");
        webcamServo.setDirection(Servo.Direction.REVERSE);
        webcamServo.setPosition(0);
        config.setResolutionX(800);
        config.setResolutionY(448);
    }

    /**
     * Gets the current angle of the camera in radians, trigonometric direction, forward is default 0
     * @return
     */
    public double getAngle()
    {
        return Math.toRadians(webcamServo.getPosition()*300);
    }

    /**
     * Sets the current angle, in radians
     * @return
     */
    public void setAngle(double angle)
    {
        webcam.getExposureControl().setExposure((long)exposureTime_microseconds, TimeUnit.MICROSECONDS);
        for(WebcamUtilsListener listener:listeners)listener.onNewAngle(angle);
        webcamServo.setPosition(Math.toDegrees(angle)/300.0);

    }

    public OpenCvWebcam getWebcam() {
        return webcam;
    }

    public CameraConfig getConfig() {
        return config;
    }
    public void start()
    {
        start(false);
    }
    public void start(boolean debug)
    {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(config.getResolutionX(), config.getResolutionY(), OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("initial camera exposure",webcam.getExposureControl().getExposure(TimeUnit.MICROSECONDS));
                webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);

            }

            @Override
            public void onError(int errorCode) {

            }
        });
        if(debug)
            FtcDashboard.getInstance().startCameraStream(webcam, 0);
    }


    public void registerListener(WebcamUtilsListener listener)
    {
        listeners.add(listener);
    }
    public void stop()
    {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}
