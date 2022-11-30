package org.firstinspires.ftc.teamcode.junctionCalibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.MovementSubsystem;
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

    public boolean active_x;
    public boolean active_y;

    Telemetry telemetry;

    public junctionAdjuster(OpenCvCamera camera_, float FOV_x, int resolution_x, int resolution_y, double diameter, Telemetry telemetry_){
        camera = camera_;

        Cam = new CameraData();
        junction = new JunctionData();

        Cam.FOV_x = FOV_x;
        Cam.resolution_x = resolution_x;
        Cam.resolution_y = resolution_y;

        active_x = true;
        active_y = true;

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

        double distance;
        double tan1,tan2;

        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();

        tan1 = (Math.tan(Math.toRadians(Cam.FOV_x/2)) / (Cam.resolution_x / 2)) * results.junction_x1;
        tan2 = (Math.tan(Math.toRadians(Cam.FOV_x/2)) / (Cam.resolution_x / 2)) * results.junction_x2;

        distance = junction.diameter / (tan2 - tan1);

        position.x = ((tan1 + tan2) / 2) * distance;
        position.y = distance;

        return position;
    }

    public Vec2 getResults(){
        Vec2 position = new Vec2();

        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();

        position.x = results.junction_x1;
        position.y = results.junction_x2;

        return position;
    }

    public void autoVisionPositioning(MovementSubsystem movementSubsystem, double speed){
        Vec2 relativePosition = relativeJunctionPosition();

        double forward = 0;
        double strafe = 0;

        if(Math.abs(relativePosition.x) > 5){
            active_x = true;

            strafe = Math.ceil(relativePosition.x/(speed * 100)) / (1/speed);

        }else if(active_x == true){
            active_x = false;

            strafe = 0;
        }

        if(Math.abs(relativePosition.y) > 5){
            active_y = true;

            forward = Math.ceil(relativePosition.y/(speed * 100)) / (1/speed);

        }else if(active_y == true){
            active_y = false;

            forward = 0;
        }

        movementSubsystem.move(forward, strafe, 0);
    }
}
