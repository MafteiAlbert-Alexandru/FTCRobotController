package org.firstinspires.ftc.teamcode.junction;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.CameraConfig;
import org.firstinspires.ftc.teamcode.vision.WebcamUtil;
import org.firstinspires.ftc.teamcode.vision.WebcamUtilsListener;
import org.openftc.easyopencv.OpenCvCamera;

public class JunctionAdjuster implements WebcamUtilsListener {
    private OpenCvCamera camera;


    public class JunctionData{
        public double diameter;
    }

    public static class visionResults{
        public Vector2d movementData;
        public boolean found;

        visionResults(Vector2d MovementData, boolean Found){
            this.movementData = MovementData;
            this.found = Found;
        }
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

    private JunctionAdjusterPipeline pipeline;

    private Vec3 relativeTransform;
    private Vec2 relativePosition;

    private double cameraTangent;
    private double cameraAngle;
    private double initialCameraAngle;
    private Vec2 cameraVec;


    private JunctionAdjusterPipeline.Results results;


    private CameraConfig config;
    private WebcamUtil webcamUtil;

    private Telemetry telemetry;

    public JunctionAdjuster(WebcamUtil webcamUtil, double diameter, Telemetry telemetry_, double initialCamAngle){
        camera = webcamUtil.getWebcam();
        config = webcamUtil.getConfig();
        junction = new JunctionData();
        this.webcamUtil = webcamUtil;

        cameraTangent = Math.tan(Math.toRadians(config.getFovX()/2));
        this.initialCameraAngle = initialCamAngle;

        junction.diameter = diameter;

        pipeline = new JunctionAdjusterPipeline();
        camera.setPipeline(pipeline);
        telemetry = telemetry_;
    }

    @Override
    public void onNewAngle(double angle) {
        this.cameraAngle = angle;
        cameraVec = new Vec2(Math.sin(angle),Math.cos(angle));
    }

    public Vec2 relativeJunctionPosition(){ // cm
        Vec2 position = new Vec2();

        double distance;
        double tan1,tan2;

        this.results = pipeline.getLatestResults();

        tan1 = (cameraTangent / (double)(config.getResolutionX() / 2.0)) * (results.junction_x1 - (double)(config.getResolutionX() / 2.0));
        tan2 = (cameraTangent / (double)(config.getResolutionX() / 2.0)) * (results.junction_x2 - (double)(config.getResolutionX() / 2.0));

        distance = junction.diameter / (tan2 - tan1);

        position.x = ((tan1+tan2)/2) * distance; //strafe
        position.y = distance; //distance

        return position;
    }

    public double relativeJunctionAngle(){
        this.results = pipeline.getLatestResults();
        double tan1,tan2;

        tan1 = (cameraTangent / (config.getResolutionX() / 2.0)) * (results.junction_x1 - (config.getResolutionX() / 2.0));
        tan2 = (cameraTangent / (config.getResolutionX() / 2.0)) * (results.junction_x2 - (config.getResolutionX() / 2.0));
        return Math.atan((tan1+tan2)/2);
    }

    public Vec3 relativeJunctionTransform(){
        Vec3 transform = new Vec3();

        double distance;
        double tan1,tan2;

        this.results = pipeline.getLatestResults();

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

        this.results = pipeline.getLatestResults();

        position.x = results.junction_x1;
        position.y = results.junction_x2;

        return position;
    }

    //this returns the strafe and forward values the robot moves by in homing state
    public visionResults value(double speed, Vec2 setPoint){
        relativePosition = relativeJunctionPosition();

        if(results.found == false){
            return new visionResults(new Vector2d(0,0),false);
        }


        Vec2 direction = new Vec2(
                relativePosition.x * cameraVec.y - relativePosition.y * cameraVec.x,
                relativePosition.y * cameraVec.y + relativePosition.x * cameraVec.x
        );

        direction = new Vec2(direction.x - setPoint.x, direction.y - setPoint.y);
        double length = Math.sqrt(direction.x* direction.x + direction.y* direction.y);

        double kp;
        if(length > 10)kp = 1;
        else kp = length/10;

        direction.x/=length;
        direction.y/=length;

        double newCamAngle = cameraAngle - (Math.atan(relativePosition.x/relativePosition.y)/15);

        if(-Math.toDegrees(newCamAngle) < 0 && -Math.toDegrees(newCamAngle) > -45){
            webcamUtil.setAngle(newCamAngle);
        }

        return new visionResults(new Vector2d(direction.x*speed*kp, direction.y*speed*kp*0.4), true);
    }
}
