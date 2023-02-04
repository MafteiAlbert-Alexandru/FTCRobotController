package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagUtil;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;


@Config
public class AutonomusOpMode extends LinearOpMode {

    public static boolean debug = false;

    TrajectorySequence leftTraj, midTraj, rightTraj;

    Pose2d LeftStartPose = new Pose2d(36, 60, Math.toRadians(90));
    Pose2d RightStartPose = new Pose2d(36, -60, Math.toRadians(90));

    private SampleMecanumDriveCancelable autoDrive;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        try {

            autoDrive = new SampleMecanumDriveCancelable(hardwareMap);
            AprilTagUtil aprilTagUtil = new AprilTagUtil(this, autoDrive);
            if(debug){
                telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
                FtcDashboard.getInstance().startCameraStream(camera, 10);
            }

            Initialization();

            waitForStart();

            RunTrajectoryCase(aprilTagUtil);
            PreActiveOpMode();

            List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
            while (opModeIsActive()) {
                for(LynxModule module:modules) module.clearBulkCache();
                autoDrive.update();
                try {
                    ActiveOpModeLoop();
                } catch (InterruptedException e) {e.printStackTrace();}
                telemetry.update();
            }

        }
        catch (Exception e) {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }

    public void Initialization(){

    }
    public void PreActiveOpMode(){

    }
    public void ActiveOpModeLoop () throws InterruptedException {

    }

    public void SetTrajectorySequence(TrajectorySequence left, TrajectorySequence mid, TrajectorySequence right){
        leftTraj = left;
        midTraj = mid;
        rightTraj = right;
    }

    public void StartPosition(Pose2d start){
        autoDrive.setPoseEstimate(start);
    }

    void RunTrajectoryCase(AprilTagUtil aprilTagUtil){
        int id = aprilTagUtil.getId();
        SampleMecanumDriveCancelable driveCancelable = aprilTagUtil.sampleMecanumDriveCancelable;
        if(id == 3) driveCancelable.followTrajectorySequenceAsync(rightTraj);
        else if(id == 2) driveCancelable.followTrajectorySequenceAsync(midTraj);
        else driveCancelable.followTrajectorySequenceAsync(leftTraj);
    }

}
