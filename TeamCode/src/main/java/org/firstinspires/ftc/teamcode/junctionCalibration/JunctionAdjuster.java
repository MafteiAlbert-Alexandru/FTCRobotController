package org.firstinspires.ftc.teamcode.junctionCalibration;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.MovementSubsystem;
import org.firstinspires.ftc.teamcode.vision.CameraConfig;
import org.firstinspires.ftc.teamcode.vision.WebcamUtil;
import org.firstinspires.ftc.teamcode.vision.WebcamUtilsListener;
import org.openftc.easyopencv.OpenCvCamera;

public class JunctionAdjuster implements WebcamUtilsListener {
    private OpenCvCamera camera;


    public class JunctionData{
        public double diameter;
    }

    public static class Vec2{
        public double x;
        public double y;

        public Vec2(){
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


    private JunctionData junction;

    private junctionAdjusterPipeline pipeline;

    private Vec3 relativeTransform;
    private double cameraTangent;
    private double cameraAngle;

    private CameraConfig config;
    private WebcamUtil webcamUtil;

    private Telemetry telemetry;

    public JunctionAdjuster(WebcamUtil webcamUtil, double diameter, Telemetry telemetry_){
        camera = webcamUtil.getWebcam();
        config = webcamUtil.getConfig();
        junction = new JunctionData();
        this.webcamUtil = webcamUtil;

        cameraTangent = Math.tan(Math.toRadians(config.getFovX()/2));

        junction.diameter = diameter;

        pipeline = new junctionAdjusterPipeline();
        camera.setPipeline(pipeline);
        telemetry = telemetry_;
    }

    @Override
    public void onNewAngle(double angle) {
        cameraAngle = Math.toDegrees(angle);
    }

    public Vec2 relativeJunctionPosition(){ // cm
        Vec2 position = new Vec2();

        double distance;
        double tan1,tan2;

        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();

        tan1 = (cameraTangent / (config.getResolutionX() / 2.0)) * (results.junction_x1 - (config.getResolutionX() / 2.0));
        tan2 = (cameraTangent / (config.getResolutionX() / 2.0)) * (results.junction_x2 - (config.getResolutionX() / 2.0));

        distance = junction.diameter / (tan2 - tan1);

        position.x = ((tan1+tan2)/2) * distance;
        position.y = distance;

        return position;
    }

    public double relativeJunctionAngle(){
        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();
        double tan1,tan2;

        tan1 = (cameraTangent / (config.getResolutionX() / 2.0)) * (results.junction_x1 - (config.getResolutionX() / 2.0));
        tan2 = (cameraTangent / (config.getResolutionX() / 2.0)) * (results.junction_x2 - (config.getResolutionX() / 2.0));
        return Math.atan((tan1+tan2)/2);
    }

    public Vec3 relativeJunctionTransform(){
        Vec3 transform = new Vec3();

        double distance;
        double tan1,tan2;

        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();

        tan1 = (cameraTangent / (double)(config.getResolutionX() / 2.0)) * (results.junction_x1 - (double)(config.getResolutionX() / 2.0));
        tan2 = (cameraTangent / (double)(config.getResolutionX() / 2.0)) * (results.junction_x2 - (double)(config.getResolutionX() / 2.0));

        distance = junction.diameter / (tan2 - tan1);

        transform.x = ((tan1+tan2)/2) * distance; //strafe
        transform.y = distance; //distance
        transform.z = Math.toDegrees(Math.atan((tan1+tan2)/2)); //angle

        return transform;
    }

    public Vec2 getResults(){
        Vec2 position = new Vec2();

        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();

        position.x = results.junction_x1;
        position.y = results.junction_x2;

        return position;
    }

    public Vector2d value(double speed, Vec2 setPoint){
        relativeTransform = relativeJunctionTransform();

        Vec2 direction = new Vec2(Math.sin(Math.toRadians(relativeTransform.z - cameraAngle)), Math.cos(Math.toRadians(relativeTransform.z - cameraAngle)));

        double length = Math.sqrt(relativeTransform.x*relativeTransform.x + relativeTransform.y*relativeTransform.y);
        direction.x *= length;
        direction.y *= length;


        direction = new Vec2(direction.x - setPoint.x, direction.y - setPoint.y);
        length = Math.sqrt(direction.x* direction.x + direction.y* direction.y);

        double kp;
        if(length > 10)kp = 1;
        else kp = length/10;

        direction.x/=length;
        direction.y/=length;

        if(relativeTransform.z - cameraAngle < 0 && relativeTransform.z - cameraAngle > -45){
            webcamUtil.setAngle(Math.toRadians(cameraAngle - (relativeTransform.z/20)));
        }

        return new Vector2d(direction.x*speed*kp, direction.y*speed*kp*0.4);
    }
}
