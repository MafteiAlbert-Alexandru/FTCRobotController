package org.firstinspires.ftc.teamcode.junctionCalibration;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.CameraConfig;
import org.firstinspires.ftc.teamcode.vision.WebcamUtil;
import org.openftc.easyopencv.OpenCvCamera;

public class PixelJunctionAdjuster {
    private OpenCvCamera camera;


    public class JunctionData{
        public double diameter;
    }
    private junctionAdjusterPipeline pipeline;
    private CameraConfig config;

    private Telemetry telemetry;

    public PixelJunctionAdjuster(WebcamUtil webcamUtil, Telemetry telemetry){
        camera= webcamUtil.getWebcam();
        config=webcamUtil.getConfig();
        pipeline = new junctionAdjusterPipeline();
        camera.setPipeline(pipeline);
        this.telemetry = telemetry;
    }
    private double angleCos;
    private double angleSin;

    public void setAngle(double angle)
    {
        angleCos=Math.cos(angle);
        angleSin=Math.sin(angle);
    }

    public Vector2d getDirection()
    {

    }
}
