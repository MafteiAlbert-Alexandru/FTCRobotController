package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class WebcamUtil {
    private Servo webcamServo;
    private OpenCvWebcam webcam;
    private CameraConfig config;
    public WebcamUtil(HardwareMap hardwareMap)
    {
        config=CameraConfig.C920;
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));
        webcamServo=hardwareMap.get(Servo.class, "webcamServo");
        webcamServo.setPosition(0);
        webcamServo.setDirection(Servo.Direction.REVERSE);
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
                webcam.startStreaming(config.getResolutionX(), config.getResolutionY());
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        if(debug)
            FtcDashboard.getInstance().startCameraStream(webcam, 0);
    }
    public void stop()
    {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}
