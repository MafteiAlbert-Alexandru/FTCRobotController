package org.firstinspires.ftc.teamcode.junctionCalibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class junctionAdjuster {
    private OpenCvCamera camera;

    public class CameraData{
        public int resolution_x;
        public int resolution_y;
        public double FOV_x;
    }

    public class JunctionData{
        public double diameter;
    }

    public static class Vec2{
        public double x;
        public double y;
    }


    private CameraData Cam;
    private JunctionData junction;

    private junctionAdjusterPipeline pipeline;

    Telemetry telemetry;

    public junctionAdjuster(OpenCvCamera camera_, float FOV_x, int resolution_x, int resolution_y, double diameter, Telemetry telemetry_){
        camera = camera_;

        Cam = new CameraData();
        junction = new JunctionData();

        Cam.FOV_x = FOV_x;
        Cam.resolution_x = resolution_x;
        Cam.resolution_y = resolution_y;

        junction.diameter = diameter;

        pipeline = new junctionAdjusterPipeline();
        camera.setPipeline(pipeline);

        telemetry = telemetry_;

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(  Cam.resolution_x, Cam.resolution_y, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {

                // This will be called if the camera could not be opened

            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);

    }
    public void stop()
    {
        FtcDashboard.getInstance().stopCameraStream();
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    public Vec2 relativeJunctionPosition(){ // cm
        Vec2 position = new Vec2();

        double angle;
        double distToJunction;

        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();

        angle = Math.toRadians(Math.abs((double)(results.junction_x1 + results.junction_x2) / (double)(Cam.resolution_x * 2) - 1) * Cam.FOV_x); // unghiul dintre vectorul de orientare a robotului si vectorul robot-junction
        distToJunction = junction.diameter / Math.tan(angle);

        position.x = Math.copySign(Math.tan(angle) * distToJunction, (double)(results.junction_x1 + results.junction_x2) / (double)(Cam.resolution_x * 2) - 1);
        position.y = distToJunction;

        return position;
    }

    public Vec2 getResults(){
        Vec2 position = new Vec2();

        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();

        position.x = results.junction_x1;
        position.y = results.junction_x2;

        return position;
    }
}
