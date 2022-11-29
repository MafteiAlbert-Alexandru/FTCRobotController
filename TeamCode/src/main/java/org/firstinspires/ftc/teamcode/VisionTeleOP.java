package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.junctionCalibration.junctionAdjusterPipeline;
import org.firstinspires.ftc.teamcode.subsystem.MovementSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.junctionCalibration.junctionAdjuster;

import java.lang.reflect.MalformedParameterizedTypeException;

@Config
@TeleOp
public class VisionTeleOP extends LinearOpMode {

    OpenCvCamera webcam;
    junctionAdjuster j_adjuster;
    public static PIDFCoefficients xCoefficients = new PIDFCoefficients(0,0,0,0.0005);
    public static PIDFCoefficients widthCoefficients = new PIDFCoefficients(0,0,0,0.0005);

    private MovementSubsystem movementSubsystem = new MovementSubsystem();
    public static double kX=0.0005;
    public static double kY=0.0005;
    @Override
    public void runOpMode(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{

            movementSubsystem.initSubsystem(this, hardwareMap);
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));
            PIDFController xController = new PIDFController(xCoefficients.p, xCoefficients.i,xCoefficients.f,xCoefficients.f);
            PIDFController widthController = new PIDFController(widthCoefficients.p, widthCoefficients.i, widthCoefficients.f);

            float FOV_x = 78;
            int resolution_x = 800;
            int resolution_y = 448;
            double diameter = 2.54;   //cm

            j_adjuster = new junctionAdjuster(webcam, FOV_x, resolution_x, resolution_y, diameter, telemetry);

            waitForStart();

            while(opModeIsActive()&&!isStopRequested()){
                xController.
                junctionAdjuster.Vec2 junctionPosition_c = j_adjuster.getResults();

                telemetry.addData("x", junctionPosition_c.x);
                telemetry.addData("width", junctionPosition_c.y- junctionPosition_c.x);
                junctionAdjuster.Vec2 target=new junctionAdjuster.Vec2();
                target.x=kX*(300-junctionPosition_c.x);
                target.y=kY*(160-(junctionPosition_c.y- junctionPosition_c.x));

                telemetry.addData("target x", 300);
                telemetry.addData("target width", 160);
                telemetry.addData("vec x", target.x);
                telemetry.addData("vec y", target.y);
                telemetry.update();
                if()
                movementSubsystem.move(target.y, target.x, 0);

            }
            j_adjuster.stop();
        }catch (Exception e)
        {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }
}