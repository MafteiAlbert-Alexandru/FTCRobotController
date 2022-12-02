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

        Vec2(){
            this.x=0;
            this.y=0;
        };
        Vec2(double x,double y){
            this.x = x;
            this.y = y;
        }
    }
    public static class Vec3{
        public double x;
        public double y;
        public double z;

        Vec3(){
            this.x=0;
            this.y=0;
            this.z=0;
        };
        Vec3(double x,double y,double z){
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }


    private CameraData Cam;
    private JunctionData junction;

    private junctionAdjusterPipeline pipeline;

    private Vec2 relativePosition;
    private Vec2 setPoint = new Vec2(-2,8.5);
    //-2,8.5
    private double cameraAngle = 45;
    public boolean active;

    Telemetry telemetry;

    public junctionAdjuster(OpenCvCamera camera_, float FOV_x, int resolution_x, int resolution_y, double diameter, Telemetry telemetry_){
        camera = camera_;

        Cam = new CameraData();
        junction = new JunctionData();

        Cam.FOV_x = FOV_x;
        Cam.resolution_x = resolution_x;
        Cam.resolution_y = resolution_y;

        active = false;

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

        tan1 = (Math.tan(Math.toRadians(Cam.FOV_x/2)) / (Cam.resolution_x / 2)) * (results.junction_x1 - (Cam.resolution_x / 2));
        tan2 = (Math.tan(Math.toRadians(Cam.FOV_x/2)) / (Cam.resolution_x / 2)) * (results.junction_x2 - (Cam.resolution_x / 2));

        distance = junction.diameter / (tan2 - tan1);

        position.x = ((tan1+tan2)/2) * distance;
        position.y = distance;

        return position;
    }

    public Vec3 relativeJunctionTransform(){
        Vec3 transform = new Vec3();

        double distance;
        double tan1,tan2;

        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();

        tan1 = (Math.tan(Math.toRadians(Cam.FOV_x/2)) / (Cam.resolution_x / 2)) * (results.junction_x1 - (Cam.resolution_x / 2));
        tan2 = (Math.tan(Math.toRadians(Cam.FOV_x/2)) / (Cam.resolution_x / 2)) * (results.junction_x2 - (Cam.resolution_x / 2));

        distance = junction.diameter / (tan2 - tan1);

        transform.x = ((tan1+tan2)/2) * distance; //strafe
        transform.y = distance; //distance
        transform.z = Math.atan((tan1+tan2)/2); //angle

        return transform;
    }

    public Vec2 getResults(){
        Vec2 position = new Vec2();

        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();

        position.x = results.junction_x1;
        position.y = results.junction_x2;

        return position;
    }

    public void autoVisionPositioning(MovementSubsystem movementSubsystem, double speed){
        relativePosition = relativeJunctionPosition();

        Vec2 direction = new Vec2(relativePosition.x - setPoint.x, relativePosition.y - setPoint.y);
        //telemetry.addData("x1", direction.x);
        //telemetry.addData("y1", direction.y);

        direction.x = direction.x * Math.cos(Math.toRadians(cameraAngle)) - direction.y * Math.sin(Math.toRadians(cameraAngle));
        direction.y = direction.x * Math.sin(Math.toRadians(cameraAngle)) + direction.y * Math.cos(Math.toRadians(cameraAngle));
        //telemetry.addData("x2", direction.x);
        //telemetry.addData("y2", direction.y);
        double xy = direction.x / direction.y;

        if(xy>1)direction = new Vec2(speed,(1/xy)*speed);
        else direction = new Vec2(xy * speed, speed);
        //telemetry.addData("x3", direction.x);
        //telemetry.addData("y3", direction.y);

        movementSubsystem.move(direction.y * 0.3, direction.x, 0);
    }
}
