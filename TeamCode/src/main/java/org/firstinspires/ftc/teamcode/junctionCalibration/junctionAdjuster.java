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
        public Vec2(double x, double y){
            this.x = x;
            this.y = y;
        }
        Vec2 normalize(){
            double length = Math.sqrt(x*x + y*y);
            x/=length;
            y/=length;
            return this;
        }
        Vec2 normalizareRapida(){
            double xy = this.x/this.y;
            if(xy>1){
                this.x = 1;
                this.y = 1/xy;
                return this;
            }
            this.x = xy;
            this.y = 1;
            return this;
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
    private Vec3 relativeTransform;
    private double relativeAngle;
    private double cameraTangent;

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
        cameraTangent = Math.tan(Math.toRadians(Cam.FOV_x/2));

        junction.diameter = diameter;

        pipeline = new junctionAdjusterPipeline();
        camera.setPipeline(pipeline);

        telemetry = telemetry_;

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //camera.startStreaming(  Cam.resolution_x, Cam.resolution_y, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {

                // This will be called if the camera could not be opened

            }
        });

        //FtcDashboard.getInstance().startCameraStream(camera, 0);

    }
    public void stop()
    {
        //FtcDashboard.getInstance().stopCameraStream();
        //camera.stopStreaming();
        camera.closeCameraDevice();
    }

    public Vec2 relativeJunctionPosition(){ // cm
        Vec2 position = new Vec2();

        double distance;
        double tan1,tan2;

        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();

        tan1 = (cameraTangent / (Cam.resolution_x / 2)) * (results.junction_x1 - (Cam.resolution_x / 2));
        tan2 = (cameraTangent / (Cam.resolution_x / 2)) * (results.junction_x2 - (Cam.resolution_x / 2));

        distance = junction.diameter / (tan2 - tan1);

        position.x = ((tan1+tan2)/2) * distance;
        position.y = distance;

        return position;
    }

    public double relativeJunctionAngle(){
        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();
        double tan1,tan2;

        tan1 = (cameraTangent / (Cam.resolution_x / 2)) * (results.junction_x1 - (Cam.resolution_x / 2));
        tan2 = (cameraTangent / (Cam.resolution_x / 2)) * (results.junction_x2 - (Cam.resolution_x / 2));
        return Math.atan((tan1+tan2)/2);
    }

    public Vec3 relativeJunctionTransform(){
        Vec3 transform = new Vec3();

        double distance;
        double tan1,tan2;

        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();

        tan1 = (cameraTangent / (Cam.resolution_x / 2)) * (results.junction_x1 - (Cam.resolution_x / 2));
        tan2 = (cameraTangent / (Cam.resolution_x / 2)) * (results.junction_x2 - (Cam.resolution_x / 2));

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

    public void autoVisionPositioning(MovementSubsystem movementSubsystem, double speed, Vec2 setPoint, double cameraAngle, double treshold){
        relativeTransform = relativeJunctionTransform();

        Vec2 direction = new Vec2(Math.sin(relativeTransform.z - cameraAngle), Math.cos(relativeTransform.z - cameraAngle));

        double length = Math.sqrt(relativeTransform.x*relativeTransform.x + relativeTransform.y*relativeTransform.y);
        direction.x *= length;
        direction.y *= length;

        direction = new Vec2(direction.x - setPoint.x, direction.y - setPoint.y);

        /*if(direction.y < 5)direction.y = 0;

        if(Math.abs(Math.sqrt(direction.x* direction.x + direction.y* direction.y)) < 1){
            movementSubsystem.move(0, 0, 0);
            return false;
        }*/

        length = Math.sqrt(direction.x* direction.x + direction.y* direction.y);

        double kp;
        if(length > 10)kp = 1;
        else kp = length/10;

        direction.x/=length;
        direction.y/=length;

        telemetry.addData("x", direction.x);
        telemetry.addData("y", direction.y);
        telemetry.addData("length", length);


        movementSubsystem.move(direction.y * 0.4 * speed * kp, direction.x * speed * kp, 0);
    }
}
