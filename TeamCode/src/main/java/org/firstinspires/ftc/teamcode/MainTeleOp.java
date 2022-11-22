package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.junctionCalibration.junctionAdjusterPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.junctionCalibration.junctionAdjuster;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    OpenCvCamera webcam;
    junctionAdjuster j_adjuster;


    @Override
    public void runOpMode(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{

            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));


            float FOV_x = 60;
            int resolution_x = 320;
            int resolution_y = 240;
            double diameter = 2.54;   //cm

            j_adjuster = new junctionAdjuster(webcam, FOV_x, resolution_x, resolution_y, diameter, telemetry);

            waitForStart();
            while(opModeIsActive()&&!isStopRequested()){
                junctionAdjuster.Vec2 junctionPosition = j_adjuster.relativeJunctionPosition();

                telemetry.addData("x ", junctionPosition.x);
                telemetry.addData("y ", junctionPosition.y);
                telemetry.update();
            }
        }catch (Exception e)
        {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }
}