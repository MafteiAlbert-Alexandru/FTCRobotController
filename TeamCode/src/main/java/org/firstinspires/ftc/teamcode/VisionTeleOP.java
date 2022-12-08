//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.subsystem.MovementSubsystem;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.firstinspires.ftc.teamcode.junctionCalibration.JunctionAdjuster;
//
//@Config
//@TeleOp
//public class VisionTeleOP extends LinearOpMode {
//
//    OpenCvCamera webcam;
//    JunctionAdjuster j_adjuster;
//
//    private MovementSubsystem movementSubsystem = new MovementSubsystem();
//
//    public static double speed = 0;
//    public static JunctionAdjuster.Vec2 setPoint = //-2,8.5
//    public static double cameraAngle = 45;
//    public static double treshold = 200;
//
//    @Override
//    public void runOpMode(){
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        try{
//
//            movementSubsystem.initSubsystem(this, hardwareMap);
//            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));
//
//            float FOV_x = 78;
//            int resolution_x = 800;
//            int resolution_y = 448;
//            double diameter = 2.54;   //cm
//
//            j_adjuster = new JunctionAdjuster(webcam, FOV_x, resolution_x, resolution_y, diameter, telemetry);
//
//            waitForStart();
//
//            while(opModeIsActive()&&!isStopRequested()){
//                j_adjuster.autoVisionPositioning(movementSubsystem, speed, setPoint, cameraAngle, treshold);
//                telemetry.update();
//            }
//            j_adjuster.stop();
//        }catch (Exception e)
//        {
//            telemetry.addLine(e.toString());
//            telemetry.update();
//        }
//    }
//}
