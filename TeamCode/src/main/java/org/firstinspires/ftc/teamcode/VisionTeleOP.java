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

    private MovementSubsystem movementSubsystem = new MovementSubsystem();

    @Override
    public void runOpMode(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try{

            movementSubsystem.initSubsystem(this, hardwareMap);
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));

            float FOV_x = 78;
            int resolution_x = 800;
            int resolution_y = 448;
            double diameter = 2.54;   //cm

            j_adjuster = new junctionAdjuster(webcam, FOV_x, resolution_x, resolution_y, diameter, telemetry);

            waitForStart();

            while(opModeIsActive()&&!isStopRequested()){
                junctionAdjuster.Vec2 junctionPosition_c = j_adjuster.relativeJunctionPosition();

                telemetry.addData("x", junctionPosition_c.x);
                telemetry.addData("y", junctionPosition_c.y);
                telemetry.update();
            }
            j_adjuster.stop();
        }catch (Exception e)
        {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }
}