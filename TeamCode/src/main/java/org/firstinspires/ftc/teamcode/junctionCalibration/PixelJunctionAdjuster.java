//package org.firstinspires.ftc.teamcode.junctionCalibration;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.control.PIDFController;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.vision.CameraConfig;
//import org.firstinspires.ftc.teamcode.vision.WebcamUtilsListener;
//import org.firstinspires.ftc.teamcode.vision.WebcamUtil;
//import org.openftc.easyopencv.OpenCvCamera;
//
//@Config
//public class PixelJunctionAdjuster implements WebcamUtilsListener {
//    private OpenCvCamera camera;
//
//    private PixelJunctionAdjusterPipeline pipeline;
//    private CameraConfig config;
//    private Telemetry telemetry;
//    public static double targetWidth=119;
//    public static double targetCenterPoint=223;
//    public static PIDCoefficients widthCoefficients = new PIDCoefficients(-0.0001,0,0);
//    public static PIDCoefficients centerPointCoefficients = new PIDCoefficients(0.0001,0,0);
//    private  PIDFController widthController,centerPointController;
//    public PixelJunctionAdjuster(WebcamUtil webcamUtil, Telemetry telemetry){
//        camera= webcamUtil.getWebcam();
//        config=webcamUtil.getConfig();
//        pipeline = new PixelJunctionAdjusterPipeline();
//        camera.setPipeline(pipeline);
//        this.telemetry = telemetry;
//        widthController = new PIDFController(widthCoefficients);
//
//        centerPointController = new PIDFController(centerPointCoefficients);
//    }
//
//    public Vector2d getDirection()
//    {
//
//        PixelJunctionAdjusterPipeline.Results results = pipeline.getLatestResults();
//
//        if(results!=null)
//        {
//            telemetry.addData("width", results.width);
//            telemetry.addData("centerPoint", results.centerPoint);
//            widthController.setTargetPosition(targetWidth);
//            centerPointController.setTargetPosition(targetCenterPoint);
//
//            Vector2d direction = new Vector2d(centerPointController.update(results.centerPoint),widthController.update(results.width));
//            double length = direction.norm();
//            if(length>1)
//                direction = direction.div(length);
//
//            telemetry.addData("directionX", direction.getX());
//            telemetry.addData("directionY", direction.getY());
//
//            direction=new  Vector2d(direction.getX()*angleCos-direction.getY()*angleSin,
//                    direction.getX()*angleSin+direction.getY()*angleCos);
//
//            telemetry.addData("adjustingX", direction.getX());
//            telemetry.addData("adjustingY", direction.getY());
//            return direction;
//        }else return new Vector2d(0,0);
//
//    }
//    public void reset()
//    {
//        widthController=new PIDFController(widthCoefficients);
//        centerPointController=new PIDFController(centerPointCoefficients);
//    }
//
//
//    private double angleCos=0;
//    private double angleSin=0;
//
//    @Override
//    public void onNewAngle(double angle) {
//        angleCos=Math.cos(angle);
//        angleSin=Math.sin(angle);
//    }
//}
