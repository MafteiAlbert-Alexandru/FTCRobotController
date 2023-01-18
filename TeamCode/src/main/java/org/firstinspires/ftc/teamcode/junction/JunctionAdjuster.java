package org.firstinspires.ftc.teamcode.junction;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.experiments.PIDController;
import org.firstinspires.ftc.teamcode.experiments.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.CameraConfig;
import org.firstinspires.ftc.teamcode.vision.WebcamUtil;
import org.firstinspires.ftc.teamcode.vision.WebcamUtilsListener;
import org.openftc.easyopencv.OpenCvCamera;

@Config
public class JunctionAdjuster implements WebcamUtilsListener {
    private OpenCvCamera camera;


    public class JunctionData{
        public double diameter;
    }

    public class visionResults{
        public Vector2d movementData;
        public double turn;
        public boolean found;

        visionResults(Vector2d MovementData, double turn, boolean Found){
            this.movementData = MovementData;
            this.turn = turn;
            this.found = Found;
        }
    }


    public class Vec3{
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
    public static Vec2 cameraVec=new Vec2();

    public static PIDCoefficients MovementCoefficients_x = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients MovementCoefficients_y = new PIDCoefficients(0.45, 0, -0.1);
    public static PIDCoefficients MovementCoefficients_turn = new PIDCoefficients(3, 0, 0);

    public static PIDCoefficients CameraCoefficients = new PIDCoefficients(2.3, 0.05, 0.3);

    private PIDController MovementController_x;
    private PIDController MovementController_y;
    private PIDController MovementController_turn;
    private PIDController CameraController;
    public static Vec2 setPoint = new Vec2(-8.2, 4.4);

    public static double reach = 2;


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
        cameraVec= new Vec2(Math.sin(0), Math.cos(0));

        CameraController = new PIDController(CameraCoefficients);
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

        tan1 = (cameraTangent / (config.getResolutionX() / 2.0)) * (results.junction_x1 - (double)(config.getResolutionX() / 2.0));
        tan2 = (cameraTangent / (config.getResolutionX() / 2.0)) * (results.junction_x2 - (double)(config.getResolutionX() / 2.0));

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

    public void start(){
        this.MovementController_x = new PIDController(MovementCoefficients_x);
        this.MovementController_y = new PIDController(MovementCoefficients_y);
        this.MovementController_turn = new PIDController(MovementCoefficients_turn);
    }

    public boolean inReach(){

        if(results==null||!results.found){return false;}

        Vec2 direction = new Vec2(
                relativePosition.x * cameraVec.y - relativePosition.y * cameraVec.x,
                relativePosition.y * cameraVec.y + relativePosition.x * cameraVec.x
        );

        direction = new Vec2(direction.x - setPoint.x, direction.y - setPoint.y);
        double length = Math.sqrt(direction.x * direction.x + direction.y * direction.y);

        return length < reach;
    }

    public void moveCamera(){
        double relativeAngle = relativeJunctionAngle();

        if(results.found == false){
            return;
        }

        double camError = -relativeAngle;
        double newCamAngle = cameraAngle+CameraController.update(camError);
        boolean camInRange = 0<Math.toDegrees(newCamAngle) && Math.toDegrees(newCamAngle) < 45;

        if(camInRange){
            webcamUtil.setAngle(newCamAngle);
        }

    }

    //this returns the strafe and forward values the robot moves by in homing state
    public visionResults value(){
        relativePosition = relativeJunctionPosition();

        if(results.found == false){
            return new visionResults(new Vector2d(0,0), 0, false);
        }

        double camError = -Math.atan(relativePosition.x/relativePosition.y);
        double newCamAngle = cameraAngle+CameraController.update(camError);
        boolean camInRange = 0<Math.toDegrees(newCamAngle) && Math.toDegrees(newCamAngle) < 45;

        if(camInRange){
            webcamUtil.setAngle(newCamAngle);
        }


        Vec2 direction = new Vec2(
                relativePosition.x * cameraVec.y - relativePosition.y * cameraVec.x,
                relativePosition.y * cameraVec.y + relativePosition.x * cameraVec.x
        );

        direction = new Vec2(direction.x - setPoint.x, direction.y - setPoint.y);

        double turn = MovementController_turn.update(Math.atan(direction.x / direction.y));


        if((this.results.junction_x1 <= 1 || this.results.junction_x2 >= config.getResolutionX() - 1) && camInRange){
            return new visionResults(new Vector2d(0,0), turn, false);
        }

        direction.x = MovementController_x.update(direction.x);
        direction.y = MovementController_y.update(direction.y);

        return new visionResults(new Vector2d(direction.x, direction.y), turn, true);
    }
}
